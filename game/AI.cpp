#include "StdAfx.h"
#include "AI.h"
#include "Weapon.h"
#include "Ships.h"
#include "GameZone.h"
#include "Sector.h"
#include "BlockPattern.h"
#include "Resources.h"
#include "Agent.h"

DEFINE_CVAR(float, kAITimeStep, 0.1f);
DEFINE_CVAR(float, kAIBigTimeStep, 0.5f);
DEFINE_CVAR(float, kAISuperTimeStep, 5.f);

static DEFINE_CVAR(float, kAIDamageReproduceCooldown, 4.f);
static DEFINE_CVAR(float, kAIInitReproduceCooldown, 10.f);
static DEFINE_CVAR(float, kAIActionFollowDistance, 1000.f);

static DEFINE_CVAR(float, kBadAimErrorAngle, 0.05f);
static DEFINE_CVAR(float, kAITargetThreshold, 0.25f);
static DEFINE_CVAR(int, kAITargetMin, 500);
static DEFINE_CVAR(float, kAIDamageDefendTime, 30.f);
static DEFINE_CVAR(float, kAIPathTimout, 10.f);
static DEFINE_CVAR(float, kTargetTimeout, 5);

static DEFINE_CVAR(float, kAIBestRangeFrac, 0.5f);

static DEFINE_CVAR(float, kVelocityObstaclesCull, 5.f);

DEFINE_CVAR(int, kAIPathMaxQueries, 30);
static DEFINE_CVAR(float, kAIParentPFleetRatio, 5.f);
static DEFINE_CVAR(bool, kAIRandomizeFlags, true);

DEFINE_CVAR(int, kAIEnableNoResReproduce, 0);
static DEFINE_CVAR(int, kAIMaxTransients, 3);

static DEFINE_CVAR(int, kVelocityObstaclesSamples, 20);
static DEFINE_CVAR(int, kTeleporterSamples, 20);

static DEFINE_CVAR(bool, kAiMod, true);

static const int AI_MOD_VERSION_MAJOR = 1;
static const int AI_MOD_VERSION_MINOR = 0;

extern float kPhysicsDamping;

Obstacle::Obstacle(const BlockCluster &bc, float radius, float dmg) NOEXCEPT
{
    pos = bc.getAbsolutePos();
    vel = bc.getVel();
    rad = radius + bc.getBRadius();
    damage = dmg;
    canShootDown = true;
}

Obstacle::Obstacle(const Projectile &pr, float radius, float dmg) NOEXCEPT
{
    pos = pr.getPos();
    vel = pr.getVel();
    rad = radius + ((pr.explosive&EExplosive::PROXIMITY) ? (float)pr.explodeRadius : pr.size);
    damage = dmg;
    canShootDown = false;
}

bool Obstacle::isDangerous(float2 clPos, float2 clVel) const
{
    return intersectRayCircle(clPos, clVel - vel, pos, kVelocityObstaclesCull * rad);
}

struct ScavengeCaps {
    float maxRange       = 0.f;
    float bestEfficiency = 0.f;
    bool  bestIsFixed    = false;
    bool  initialized    = false;
};

static ScavengeCaps getScavengeCaps(const BlockCluster *self)
{
    ScavengeCaps r;
    r.maxRange       = 0.f;
    r.bestEfficiency = 0.f;
    r.bestIsFixed    = false;
    r.initialized    = true;

    ASSERT(self->command);

    foreach (const Block* bl, self->blocks)
    {
        if (!bl->sb.isWeapon())
            continue;
        float efficiency = bl->sb.weaponEfficiency();
        if (efficiency > r.bestEfficiency) {
            r.bestEfficiency = efficiency;
            r.maxRange = bl->getWeaponRange();
            r.bestIsFixed = bl->sb.isFireableFixed();
        }
    }
    return r;
}


float AttackCapabilities::getDpsAtRange(float range) const
{
    float dps = 0;
    foreach (auto &x, rangeDmg)
    {
        if (range <= x.first)
            dps += x.second;
        else
            break;
    }
    return dps;
}

const AttackCapabilities& AI::getAttackCaps()
{
    if (m_attackCaps.initialized)
        return m_attackCaps;

    const float sensorRad = command->cluster->getSensorRadius();
    uint  weaponCount = 0;
    AttackCapabilities &r = m_attackCaps;
    r = AttackCapabilities();
    r.initialized = true;
    r.maxAccel = nav->maxAccel.x;

    foreach (const Block* bl, command->cluster->blocks)
    {
        r.totalHealth += bl->sb.health;
        if (bl->shield) {
            r.totalHealth += bl->shield->health;
            r.healthRegen += bl->sb.shield->regen;
        }
        r.healthRegen += 0.25f * bl->getHealRate();
        const float dmg = bl->sb.weaponDamagePerSec();
        if (dmg <= 0.f)
            continue;
        const float range = min(sensorRad, bl->sb.offset.x + bl->getWeaponRange());
        const Feature_t ftrs = bl->sb.features.get();

        r.maxRange  = max(r.maxRange, range);
        r.weapons  |= ftrs;
        // don't aim based on fixed weapons that are not pointing forwards
        r.hasFixed |= (bl->sb.isFireableFixed() && fabsf(distanceAngles(bl->sb.angle, 0.f)) < 0.1f);
        // launchers don't count as DPS since they are so indirect
        r.totalDps += dmg;
        r.rushDps += (ftrs&(Block::LAUNCH|Block::LAUNCHER)) ? 0.f : dmg;
        if ((ftrs&Block::CANNON) && (ftrs&Block::TURRET)) {
            r.weaponVel = max(bl->getWeaponVel(), r.weaponVel);
        }
        if (ftrs&Block::AUTOFIRE)
            r.autofireDps += dmg;

        bool foundIt = false;
        foreach (auto &x, r.rangeDmg) {
            if (abs(range - x.first) < 50.f) {
                x.second += dmg;
                foundIt = true;
                break;
            }
        }
        if (!foundIt)
            r.rangeDmg.push_back(make_pair(range, dmg));
        weaponCount++;
    }

    if (!weaponCount)
        return r;

    // best range is farthest where we get most of our total damage
    vec_sort(r.rangeDmg, [](const pair<float, float>& a, const pair<float, float>& b) { return a.first > b.first; });
    float accumDmg = 0;
    foreach (const auto& x, r.rangeDmg) {
        accumDmg += x.second;
        if (accumDmg > kAIBestRangeFrac * r.totalDps) {
            r.bestRange = x.first;
            break;
        }
    }

    return r;
}

bool FiringFilter::allow(const Block *bl) const
{
    if (minEfficiency > 0.f && bl->sb.weaponEfficiency() < minEfficiency)
        return false;
    if ((bl->sb.features&allowMask) != bl->sb.features.get())
        return false;
    if (healer && !bl->sb.isHealer())
        return false;
    return true;
}


FiringData::FiringData(const Block *bl) :
    command(bl),
    pos(bl->getAbsolutePos()),
    clusterPos(bl->cluster->getAbsolutePos()),
    clusterVel(bl->cluster->getAbsoluteVel()),
    clusterRad(bl->cluster->getCoreRadius()),
    faction(bl->cluster->getFaction())
{
}

int AI::fireWeaponsAt(FiringData &data)
{
    const BlockCluster *mycl = command->cluster;

    if (mycl->dirty ||
        mycl->version != m_weaponsVersion)
    {
        const_cast<BlockCluster*>(mycl)->applyBlueprintBindings();
        m_weapons.blocks.clear();
        foreach (Block* bl, mycl->blocks)
        {
            if (bl->sb.isWeapon() && (bl->sb.features&FIREABLE_WEAPONS))
                m_weapons.blocks.push_back(bl);
        }
        m_weapons.init(m_config.flags&SerialCommand::RIPPLE_FIRE,
                       m_config.flags&SerialCommand::SPREAD_FIRE);

        m_weaponsVersion = mycl->version;
        // bibi: iiiiiiiii    }errrrrrrrrrrrr8888888888]\vvvvvvvv
    }

    data.aimError  = (m_config.flags&SerialCommand::BAD_AIM) ? randrange(-kBadAimErrorAngle, kBadAimErrorAngle) : 0.f;
    data.totalSpread = m_weapons.spreadCount ?
                       spreadCircleToCircle(mycl->getAbsolutePos(), mycl->getCoreRadius(),
                                            data.clusterPos, data.clusterRad) : 0.f;

    // intelligently turn on spread fire if the target might dodge
    const AI *tai = data.command->commandAI.get();
    if ((m_config.flags&SerialCommand::SMART_FIRE) && tai)
    {
        const AttackCapabilities &caps = getAttackCaps();

        const float targetDist = distance(data.clusterPos, mycl->getPos());
        const float dodgeTime = max(0.f, (targetDist / caps.weaponVel - kAITimeStep));
        const float dodgeDist = 0.5f * length(tai->nav->maxAccel) * dodgeTime * dodgeTime * pow(kPhysicsDamping, dodgeTime);

        m_weapons.spreadCount = 0;
        if (dodgeDist >= data.clusterRad)
        {
            foreach (const Block *bl, m_weapons.blocks)
            {
                if (bl->sb.weaponRange() > targetDist && bl->sb.weaponSpreads())
                    m_weapons.spreadCount++;
            }

            const float rad = min(targetDist/4.f,
                                  min(dodgeDist,
                                      m_weapons.spreadCount * data.clusterRad/2.f));
            data.totalSpread = min(spreadCircleToCircle(mycl->getAbsolutePos(), mycl->getCoreRadius(),
                                                        data.clusterPos, rad),
                                   M_PI_4f);
        }
    }

    return m_weapons.fireAtTarget(data);
}

Block * AI::enumWeapons(int * indexInOut) {
    if (!indexInOut || *indexInOut < 0)
        return NULL;
    if (*indexInOut >= m_weapons.blocks.size())
        return NULL;

    Block * bl = m_weapons.blocks[*indexInOut];
    *indexInOut += 1;
    return bl;
}

int WeaponGroup::fireAtTarget(FiringData &data) const
{
    int enabled = 0;
    data.wg = this;

    for (int index=0; index<blocks.size(); index++)
    {
        Block *bl = blocks[index];

        if (!data.filter.allow(bl))
            continue;

        data.index = index;
        data.enableSpacialQuery = bl->cluster->zone->isUpdateSubStep(bl, kAITimeStep, kAIBigTimeStep);

        if (bl->fireWeapon(data))
        {
            enabled++;
            if (data.filter.minEfficiency > 0.f)
                break;
        }
    }

    return enabled;
}

struct NVec { float2 dir; float2 vec; float len; };

// return unnormalized direction vector to point ship for shooting at target with fixed weapons
float2 directionForFixed(const BlockCluster *cluster, float2 targetPos, float2 targetVel,
                         const FiringFilter &filter)
{
    // Bucket fixed weapons into 8 cardinal direction. Pick the average of the top bucket, or if
    // there two that are about equal, pick the one closest to the current orientation.
    NVec dirs[] = {
        { f2(1.f, 0.f),  f2(), 0.f }, { f2(.707, .707),   f2(), 0.f },
        { f2(0.f, 1.f),  f2(), 0.f }, { f2(-.707, .707),  f2(), 0.f },
        { f2(-1.f, 0.f), f2(), 0.f }, { f2(-.707, -.707), f2(), 0.f },
        { f2(0.f, -1.f), f2(), 0.f }, { f2(.707, -.707),  f2(), 0.f },
    };
        
    foreach (const Block *bl, cluster->blocks)
    {
        if (!bl->sb.isFireableFixed() || !filter.allow(bl))
            continue;
        const float dmg = abs(bl->sb.weaponDamagePerSec());
        if (dmg < epsilon)
            continue;
        const float2 dir = normalize(bl->weaponDirForTarget(targetPos, targetVel));
        float align = -1.f;
        NVec *wdir = NULL;
        foreach (NVec &nv, make_pair(dirs, dirs + arraySize(dirs))) {
            if (test_max(dot(nv.dir, dir), &align))
                wdir = &nv;
        }
        wdir->vec += dmg * dir;
        wdir->len += dmg;
    }

    vec_sort(make_pair(dirs, dirs + arraySize(dirs)), [](const NVec &a, const NVec &b) { return a.len > b.len; });
    
    if (dirs[0].len <= 1.f)
        return targetPos - cluster->getAbsolutePos();

    if (dirs[0].len > dirs[1].len * 1.5f)
    {
        // only 1 dir to consider
        return dirs[0].vec;
    }

    // 2 possible dirs, pick closest one
    const float2 currentDir = cluster->getRot();
    return (dot(dirs[0].vec, currentDir) > dot(dirs[1].vec, currentDir)) ? dirs[0].vec : dirs[1].vec;
}

static float2 directionForFixed(const BlockCluster *cluster, const FiringData &data)
{
    return directionForFixed(cluster, data.clusterPos, data.clusterVel, data.filter);
}

float2 getTargetDirection(const AI* ai, const vector<Obstacle> &obs)
{
    float2 accel = ai->nav->action.accel;
    if (!nearZero(accel))
        return normalize(accel);
    if (obs.size()) {
        float2 myPos = ai->command->getClusterPos();
        float2 avgPos = vec_average<float2>(obs, [](const Obstacle &ob) { return ob.pos; });
        if (!nearZero(myPos - avgPos))
            return normalize(myPos - avgPos);
    }
    return float2(0);
}

bool velocityObstacles(float2 *velOut, float* pBestDamage, float2 position, float2 velocity,
                              float2 maxAccel, float2 targetDir, float2 rushDir, const vector<Obstacle> &obstacles)
{
    if (!obstacles.size())
        return false;
    float2 bestVel         = float2(0, 0);
    float  bestVelScore    = std::numeric_limits<float>::lowest();
    float  bestVelDirScore = std::numeric_limits<float>::lowest();
    float  defaultDamage   = 0;
    float  bestDamage      = 0;

    // velocity obstacles!
    const float deltaV = 0.25f * length(maxAccel); // FIXME we are assuming a quarter second to impact

    for (uint i=0; i<kVelocityObstaclesSamples; i++)
    {
        const float2 sdir = ((i == 0) ? float2(0, 0) :
                             ((i == 1) ? targetDir :
                              angleToVector(randangle())));
        if (dot(rushDir, sdir) < -epsilon)
            continue;
        const float2 sample = velocity + deltaV * sdir;

        float sampleScore = 0.f;
        foreach (const Obstacle &o, obstacles)
        {
            if (!nearZero(sample - o.vel) &&
                intersectRayCircle(position, sample - o.vel, o.pos, o.rad))
            {
                sampleScore -= o.damage;
            }
        }

        const float sampleDirScore = dot(sample, targetDir);
        const bool  betterDir      = sampleScore == bestVelScore && sampleDirScore > bestVelDirScore;

        if (sampleScore > bestVelScore || betterDir)
        {
            // first run is current velocity - if we are collision free then we can ignore the obstacles
            if (i == 0) {
                if (sampleScore == 0.f)
                    return false;
                else
                    defaultDamage = sampleScore;
            }

            bestVel = sample;
            bestDamage = sampleScore;
            if (betterDir)
                bestVelDirScore = sampleDirScore;
            else
                bestVelScore = sampleScore;
        }
    }

    if (pBestDamage)
        *pBestDamage = bestDamage;

    // only avoid if we can reduce damage
    if (nearZero(bestVel) || bestDamage <= 0.9f * defaultDamage)
        return false;

    *velOut = bestVel;
    return true;
}

/*
struct ObstacleQuery
{
private:
    vector<Projectile*> m_projVec;
    vector<Block*>      m_blockVec;
    vector<Obstacle>    m_obstacles;

public:

    const vector<Obstacle> &getLast() const { return m_obstacles; }

    const vector<Obstacle> &queryObstacles(Block* command, bool blocksOnly=false)
    {
        BlockPattern bp = patternForTarget(command, Block::EXPLODE|Block::MELEE);
        bp.hasCommand = 0;      // torpedoes, mines!
        const BlockCluster* cluster = command->cluster;
        const float2        clPos   = cluster->getAbsolutePos();
        const float2        clVel   = cluster->getVel();
        const float         clRad   = cluster->getShieldBRadius();
        GameZone*           zone    = command->cluster->zone;

        if (!blocksOnly)
        {
            m_obstacles.clear();
            m_projVec.clear();
            zone->intersectCircleProjectiles(&m_projVec, bp.position, bp.radius);
            foreach (Projectile* pr, m_projVec) {
                if (cluster->weaponCollide(pr->faction))
                {
                    Obstacle obs(*pr, clRad, pr->health);
                    if (obs.isDangerous(clPos, clVel))
                        m_obstacles.push_back(obs);
                }
            }
        }

        m_blockVec.clear();
        zone->intersectCircleBlocks(&m_blockVec, bp);
        for (int i=0; i<m_blockVec.size(); )
        {
            const Block *bl = m_blockVec[i];
            Obstacle obs(*bl->cluster, clRad, 2 * bl->sb.explodeDamage);
            if (!vec_pop_increment(m_blockVec, i, !obs.isDangerous(clPos, clVel)))
                m_obstacles.push_back(obs);
        }
        return m_obstacles;
    }

    const vector<Block*> &queryBlockObstacles(Block *command)
    {
        queryObstacles(command, true);
        return m_blockVec;
    }

    int cullForDefenses(const AttackCapabilities &caps)
    {
        vec_sort_key(m_obstacles, [](const Obstacle& obs) { return -obs.damage; });
        const float interval = 0.25f;
        float autoDps = interval * caps.autofireDps;
        float regen  = interval * caps.healthRegen;
        int culled = 0;

        while (m_obstacles.size())
        {
            const Obstacle &obs = m_obstacles.back();
            if (obs.canShootDown && obs.damage < autoDps) {
                autoDps -= obs.damage;
                m_obstacles.pop_back();
                culled++;
            } else if (obs.damage < regen) {
                regen -= obs.damage;
                m_obstacles.pop_back();
                culled++;
            } else {
                break;
            }
        }
        return culled;
    }

};
*/

const vector<Obstacle> &ObstacleQuery::getLast() const { return m_obstacles; }

const vector<Obstacle> &ObstacleQuery::queryObstacles(Block* command, bool blocksOnly)
{
    BlockPattern bp = patternForTarget(command, Block::EXPLODE|Block::MELEE);
    bp.hasCommand = 0;      // torpedoes, mines!
    const BlockCluster* cluster = command->cluster;
    const float2        clPos   = cluster->getAbsolutePos();
    const float2        clVel   = cluster->getVel();
    const float         clRad   = cluster->getShieldBRadius();
    GameZone*           zone    = command->cluster->zone;

    if (!blocksOnly)
    {
        m_obstacles.clear();
        m_projVec.clear();
        zone->intersectCircleProjectiles(&m_projVec, bp.position, bp.radius);
        foreach (Projectile* pr, m_projVec) {
            if (cluster->weaponCollide(pr->faction))
            {
                Obstacle obs(*pr, clRad, pr->health);
                if (obs.isDangerous(clPos, clVel))
                    m_obstacles.push_back(obs);
            }
        }
    }

    m_blockVec.clear();
    zone->intersectCircleBlocks(&m_blockVec, bp);
    for (int i=0; i<m_blockVec.size(); )
    {
        const Block *bl = m_blockVec[i];
        Obstacle obs(*bl->cluster, clRad, 2 * bl->sb.explodeDamage);
        if (!vec_pop_increment(m_blockVec, i, !obs.isDangerous(clPos, clVel)))
            m_obstacles.push_back(obs);
    }
    return m_obstacles;
}

const vector<Block*> &ObstacleQuery::queryBlockObstacles(Block *command)
{
    queryObstacles(command, true);
    return m_blockVec;
}

int ObstacleQuery::cullForDefenses(const AttackCapabilities &caps)
{
    vec_sort_key(m_obstacles, [](const Obstacle& obs) { return -obs.damage; });
    const float interval = 0.25f;
    float autoDps = interval * caps.autofireDps;
    float regen  = interval * caps.healthRegen;
    int culled = 0;

    while (m_obstacles.size())
    {
        const Obstacle &obs = m_obstacles.back();
        if (obs.canShootDown && obs.damage < autoDps) {
            autoDps -= obs.damage;
            m_obstacles.pop_back();
            culled++;
        } else if (obs.damage < regen) {
            regen -= obs.damage;
            m_obstacles.pop_back();
            culled++;
        } else {
            break;
        }
    }
    return culled;
}

LoadStatus loadAiMod(FactionData* data);

void AI::setMod(const FactionData *dat, bool shouldResetActions)
{
    if (dat)
    {
        LoadStatus stat = loadAiMod(const_cast<FactionData*>(dat));
        if (stat == LS_OK)
            m_aiModCreateActions = dat->aiMod.createActions;
    }
    else
    {
        m_aiModCreateActions = nullptr;
    }

    if (shouldResetActions) {
        resetForActions();
        recreateActions();
    }
}

ObstacleQuery& AI::getQuery()
{
    if (!m_obsQuery)
        m_obsQuery = new ObstacleQuery;
    return *m_obsQuery;
}

uint AIAction::noAction(const char* reason)
{
    status = reason;
    return LANE_NONE;
}

string AIAction::toString() const
{
    return str_format("%24s(%2x|%s) ", toStringName().c_str(), Lanes,
                      Blocking ? str_format("%2x", Blocking).c_str() : "  ") + toStringEx();
}

const BlockCluster* AIAction::getCluster() const { return m_ai->command->cluster; }
float2 AIAction::getClusterPos() const { return getCluster()->getAbsolutePos(); }
float  AIAction::getClusterBRadius() const { return getCluster()->getBRadius(); }

bool AIAction::isDestObstructed(float2 dest, uint ignoreFaction=0, float avoidDebrisRatio=kAvoidDebrisRadiusRatio) const
{
    const BlockCluster *cl = getCluster();
    SegmentPattern pat(cl->getAbsolutePos(), dest,
                       max(kAvoidDebrisRadiusMin, avoidDebrisRatio * cl->getBRadius()));
    if (cl->zone->intersectSegmentBounds(pat.start, pat.end))
        return true;
    pat.ignoreCl = cl;
    pat.ignoreFaction = ignoreFaction;
    if (m_ai->command->sb.lifetime > 0.f)
        pat.weaponFaction = m_ai->getFaction();
    return cl->zone->intersectSegmentClusters(pat);
}

// repath if we hit something (bigger than kAvoidDebrisRadiusRatio)
bool AIAction::needsRepath() const
{
    return m_ai->isBigUpdate() &&
        (m_ai->zone->simTime - getCluster()->live->lastCollisionTime < kAIBigTimeStep);
}

AIMood AIAction::isActivelyHostile(const Block* target) const
{
    const AI* tai = target->commandAI.get();
    if (!tai && (target->sb.features&(Block::EXPLODE|Block::MELEE)) &&
        m_ai->command->getFaction() != target->sb.explodeFaction)
        return AI::DEFENSIVE;
    if (!live(target) || !tai)
        return AI::NEUTRAL;
    if (tai->mood == AI::OFFENSIVE && tai->moodFaction == m_ai->command->getFaction())
        return AI::DEFENSIVE;
    if ((m_ai->zone->simTime - m_ai->lastDamagedTime) < kAIDamageDefendTime &&
        tai->getFaction() == m_ai->damagedFaction)
        return AI::OFFENSIVE;
    return AI::NEUTRAL;
}

float AIAction::getWaypointRadius() const
{
    return 3.f * getClusterBRadius();
}

uint AAvoidWeapon::update(uint blockedLanes)
{
    // give other actions a chance to run sometimes
    // if (m_ai->isBigUpdate())
        // return LANE_NONE;

    const vector<Obstacle>& obs = m_ai->getQuery().queryObstacles(m_ai->command);

    count = obs.size();
    culled = m_ai->getQuery().cullForDefenses(m_ai->getAttackCaps());

    float bestDamage = 0;
    bool avoidSolution = velocityObstacles(&config.velocity, &bestDamage,
                                           getClusterPos(), getCluster()->getVel(),
                                           m_ai->nav->maxAccel, getTargetDirection(m_ai, obs),
                                           m_ai->rushDir, obs);

    ASSERT(bestDamage <= 0);

    if ((m_ai->getConfig().features&Block::TELEPORTER) && bestDamage < -30)
    {
        // teleport me somewhere
        const BlockCluster *cl = getCluster();

        const float2 pos    = cl->getAbsolutePos();
        const float2 vel    = cl->getAbsoluteVel();
        const float  brad   = cl->getBRadius();
        const float  radius = cl->command->sb.command->sensorRadius;

        const GameZone *zone   = cl->zone;

        const float2 tpos      = m_ai->getTargetPos();
        const float  tradius   = m_ai->getAttackCaps().bestRange;
        const bool   hasTarget = !nearZero(tpos) && tradius > 0.f;

        float2 bestPos;
        int    bestScore = bestDamage;

        for (int i=0; i<kTeleporterSamples; i++)
        {
            const float2 spos = pos + randpolar(brad, radius);

            if (zone->intersectPointBounds(spos) ||
                zone->intersectCircleClusterCirclesNearest(spos, brad))
                continue;

            int score = 0;

            foreach (const Obstacle &ob, obs)
            {
                if (intersectRayCircle(spos, vel - ob.vel, ob.pos, ob.rad))
                {
                    score -= ob.damage;
                }
            }

            if (hasTarget && intersectPointCircle(spos, tpos, tradius))
            {
                // teleporter is mostly defensive so only increase a little
                score += 1;
            }

            if (score > bestScore)
            {
                bestScore = score;
                bestPos = spos;
            }
        }

        if (!nearZero(bestPos))
        {
            const float angle = hasTarget ? vectorToAngle(tpos - bestPos) :
                                !nearZero(vel) ? vectorToAngle(vel) :
                                cl->getAngle();
            if (m_ai->command->cluster->teleportToDest(bestPos, angle))
                return LANE_MOVEMENT;
        }
    }

    if (avoidSolution)
    {
        uint dims = (nearZero(m_ai->rushDir) ? SN_VELOCITY|SN_VEL_ALLOW_ROTATION : SN_VELOCITY);
        m_ai->nav->setDest(config, dims, 0);
        return LANE_MOVEMENT;
    }
    return LANE_NONE;
}

void AAvoidCluster::generateClusterObstacleList(Block* command)
{
    // FIXME this should include asteroids! - nearby asteroid blocks?
    const BlockCluster *cl       = command->cluster;
    const float         clRad    = cl->getBRadius();
    const float2        clPos    = cl->getAbsolutePos();
    const float2        clVel    = cl->getVel();
    const BlockList    &commands = m_ai->getAllies();

    obstacles.clear();
    foreach (const Block *bl, commands)
    {
        const BlockCluster *bcl = bl->cluster;
        if (bl != command && cl != bcl && bcl->getMass() > 0.5f * cl->getMass())
        {
            Obstacle obs(*bcl, clRad, bcl->getMass() / cl->getMass());
            if (intersectRayCircle(clPos, clVel - obs.vel, obs.pos, 5 * obs.rad))
                obstacles.push_back(obs);
        }
    }
}

uint AAvoidCluster::update(uint blockedLanes)
{
    // explosive blocks / missiles don't collide with each other anyway
    if (m_ai->command->sb.features&Block::EXPLODE)
        return LANE_NONE;

    // don't bother for single seeds
    if ((m_ai->command->sb.features&Block::SEED) && getCluster()->size() == 1)
        return LANE_NONE;

    generateClusterObstacleList(m_ai->command);

    if (velocityObstacles(&config.velocity, NULL, getClusterPos(), getCluster()->getVel(),
                          m_ai->nav->maxAccel, getTargetDirection(m_ai, obstacles), float2(), obstacles))
    {
        m_ai->nav->setDest(config, SN_VELOCITY, 0);
        return LANE_MOVEMENT;
    }
    return LANE_NONE;
}


const BlockCluster* PathFinder::intersectSegmentClusters(const BlockCluster* cl, float2 start, float2 end) const
{
    SegmentPattern pat(start, end, max(kAvoidDebrisRadiusMin,
                                       kAvoidDebrisRadiusRatio * cl->getBRadius()));

    if ((cl->command->sb.features&(Block::EXPLODE|Block::MELEE)) && cl->getFaction() != 0) {
        const uint targetFaction = exist(cl->command->getAI()->target) ?
                                   cl->command->getAI()->target->getFaction() : 0;
        pat.weaponFaction = cl->getFaction();
        pat.ignoreFaction = (targetFaction != cl->getFaction()) ? targetFaction : 0;
    } else {
        pat.ignoreCl = cl;
    }
    return cl->zone->intersectSegmentClusters(pat);
}

bool PathFinder::trace1(const BlockCluster* ocl, const BlockCluster *cl, float2 start, float2 end,
            float g, const Node *parent)
{
    if (ocl)
    {
        if (m_traced.count(ocl))
            return false;
        m_traced.insert(ocl);

        const float  rad    = M_SQRT2f * (ocl->getBRadius() + cl->getBRadius());
        const float2 pos    = ocl->getAbsolutePos();
        const float2 vec    = rad * normalize(pos - start);
        const float2 leftp  = pos + rotate90(vec);
        const float2 rightp = pos - rotate90(vec);

        bool found_left  = trace(cl, start, leftp, g, parent);
        bool found_right = trace(cl, start, rightp, g, parent);
        if (found_left || found_right)
            return true;
        else
            return trace(cl, start, pos - vec, g, parent);
    }
    else
    {
        Node *node  = new Node;
        node->pos    = end;
        node->parent = parent;
        node->g      = parent ? parent->g + 1 : 0;
        node->f      = float(node->g) + g;
        m_q.push(node);
        m_nodes = slist_insert(m_nodes, node);
        m_explored.insertPoint(end, true);
        return true;
    }
}

bool PathFinder::trace(const BlockCluster *cl, float2 start, float2 end, float g, const Node *parent)
{
    // give up before we grind the game to a halt
    if (m_explored.intersectCircle(end, m_explored.cell_size()) ||
        ++m_queries >= m_maxQueries)
    {
        if (m_queries == m_maxQueries)
        {
            //WARN(("PathFinder giving up after %d queries", m_queries));
        }

        return false;
    }

    const BlockCluster* obs = intersectSegmentClusters(cl, start, end);
    return trace1(obs, cl, start, end, g, parent);
}

void PathFinder::calcPath(const Node *node)
{
    m_path.clear();
    for (; node != NULL; node = node->parent) {
        m_path.push_back(node->pos);
    }
    std::reverse(m_path.begin(), m_path.end());
}

void PathFinder::clear()
{
    m_path.clear();
    m_nodes = slist_clear(m_nodes);
    m_traced.clear();
    m_explored.clear();
    m_q = PQueue();
    m_queries = 0;
}

PathFinder::PathFinder() : m_explored(kComponentWidth * 5.f, 100) {}

const vector<float2>& PathFinder::findPath(const BlockCluster *cl, float2 end, float endRad, int maxc)
{
    clear();
    m_maxQueries = maxc;
    const float2 start = cl->getAbsolutePos();

    // path is out of bounds
    if (cl->zone->intersectSegmentBounds(start, end - endRad * normalize(start - end)))
        return m_path;

    const BlockCluster* obs = intersectSegmentClusters(cl, start, end);
    if (!obs || (intersectPointCircle(obs->getAbsolutePos(), end, endRad) &&
                 obs->getBRadius() < endRad))
    {
        // no obstacles - return immediately
        m_path.push_back(end);
        return m_path;
    }
    else if (obs && intersectPointCircle(start, obs->getAbsolutePos(), obs->getMinRadius()))
    {
        // destination is inside of obstacle
        return m_path;
    }

    trace1(obs, cl, start, end, distance(start, end), NULL);

    float       minDist = FLT_MAX;
    const Node *closest = NULL;

    while (!m_q.empty())
    {
        const Node *x = m_q.top();
        m_q.pop();

        if (intersectPointCircle(x->pos, end, endRad))
        {
            calcPath(x);
            return m_path;
        }
        else if (distanceSqr(x->pos, end) < minDist)
        {
            minDist = distanceSqr(x->pos, end);
            closest = x;
        }

        m_traced.clear();
        trace(cl, x->pos, end, distance(x->pos, end), x);
    }

    // better than nothing
    if (closest)
        calcPath(closest);
    return m_path;
}

void PathFinder::render(VertexPusherLine& line) const
{
    line.color(COLOR_RESOURCE0);
    foreach (const BlockCluster* cl, m_traced) {
        line.PushCircle(cl->getAbsolutePos(), cl->getBRadius());
    }
    foreach (const Node* node, slist_iter(m_nodes))
    {
        line.PushCircle(node->pos, 10);
        if (node->parent)
            line.PushLine(node->parent->pos, node->pos);
    }
}

AMove::AMove(AI* ai) : AIAction(ai, LANE_MOVEMENT)
{
    ASSERT(m_ai->getConfig().isMobile);
    IsFinished = true;
}
AMove::AMove(AI* ai, float2 pos, float r) : AIAction(ai, LANE_MOVEMENT)
{
    setMoveDest(pos, r);
}

void AMove::setMoveDest(float2 pos, float r)
{
    target = snConfigDims();
    target.dims = (m_ai->command->sb.features&(Block::EXPLODE|Block::MELEE))
                  ? SN_POSITION|SN_TARGET_VEL : SN_POSITION;
    target.cfg.position = pos;
    prec.pos = r;
    IsFinished = false;
}

void AMove::setMoveAngle(float angle, float rad)
{
    target = snConfigDims();
    target.cfg.angle = angle;
    target.dims      = SN_ANGLE;
    prec.angle       = rad;
    IsFinished       = false;
}

void AMove::setMoveRot(float2 vec, float rad)
{
    setMoveAngle(v2a(vec), rad);
}

void AMove::setMoveDestAngle(float2 pos, float r, float angle, float rad)
{
    target = snConfigDims();
    target.cfg.position = pos;
    target.cfg.angle    = angle;
    target.dims         = SN_POSITION|SN_ANGLE;
    prec.pos            = r;
    prec.angle          = rad;
    IsFinished          = false;
}

void AMove::setMoveConfig(const snConfigDims& cfg, float radius)
{
    target     = cfg;
    prec.pos   = radius;
    IsFinished = false;
}

uint AMove::update(uint blockedLanes)
{
    if (IsFinished)
        return LANE_NONE;
    IsFinished = m_ai->nav->isAtDest(target, prec);
    if (IsFinished)
        return LANE_NONE;
    m_ai->nav->setDest(target.cfg, target.dims, prec);
    return LANE_MOVEMENT;
}


void APath::setPathDest(float2 p, float r)
{
    time = m_ai->zone->simTime;
    pos = p;
    rad = r;
    IsFinished = false;
    move.IsFinished = true; // force repath
}

bool APath::isAtDest() const
{
    return intersectPointCircle(getClusterPos(), pos, rad);
}

uint APath::update(uint blockedLanes)
{
    if (IsFinished)
        return LANE_NONE;

    if (move.IsFinished ||
        needsRepath() ||
        (m_ai->zone->simTime - time > kAIPathTimout) ||
        (m_ai->isBigUpdate() && isDestObstructed(move.target.cfg.position)))
    {
        if (isAtDest())
        {
            // complete
            move.IsFinished = true;
            IsFinished = true;
            return LANE_NONE;
        }

        const int queries = clamp(ceil_int(sqrt(getCluster()->getDeadliness())),
                                  kAIPathMaxQueries / 4,
                                  kAIPathMaxQueries);

        // FIXME this needs to be much more efficient
        // FIXME 1. repath much less frequently (NEVER more than every big update)
        // FIXME 2. even if complete path not found, head towards waypoint
        const vector<float2>& waypoints = path.findPath(getCluster(), pos, rad, queries);
        if (waypoints.empty())
        {
            // no path - give up.
            move.IsFinished = true;
            IsFinished = true;
            return LANE_NONE;
        }

        move.setMoveDest(waypoints[0], (waypoints.size() == 1) ? rad : getWaypointRadius());
    }

    return move.update(0);
}

void APath::render(void* lineVoid) const
{
    VertexPusherLine& line = *(VertexPusherLine*)lineVoid;
    if (IsFinished)
        return;
    path.render(line);
    line.color(COLOR_R);
    float2 lastPos = getClusterPos();
    foreach (float2 p, path.getPath())
    {
        line.PushCircle(p, getClusterBRadius());
        line.PushLine(lastPos, p);
        lastPos = p;
    }
    line.color(COLOR_R);
    line.PushCircle(pos, rad);
}


// used for mouse diablo-esque controls on player
struct AGoto final : public AIAction {

    AMove move;

    static bool supportsConfig(const AICommandConfig& cfg) { return AMove::supportsConfig(cfg); }
    virtual void render(void* lineVoid) const { move.render(lineVoid); }

    AGoto(AI* ai) : AIAction(ai, LANE_MOVEMENT), move(ai) {}

    virtual uint update(uint blockedLanes)
    {
        snConfigDims cfg;
        cfg.dims = SN_POSITION|SN_TARGET_VEL;
        cfg.cfg.position = m_ai->command->sb.command->destination;
        move.setMoveConfig(cfg, getWaypointRadius());
        return move.update(blockedLanes);
    }
};

struct AInvestigateHereditary final : public AIAction {

    watch_ptr<const Block> pathTarget;
    APath             path;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return cfg.hasParent && APath::supportsConfig(cfg);
    }
    virtual void render(void* line) const { path.render(line); }

    AInvestigateHereditary(AI* ai) : AIAction(ai, LANE_MOVEMENT), path(ai) { }

    virtual uint update(uint blockedLanes)
    {
        if (!m_ai->getParent())
            return noAction("No Parent");

        if (m_ai->isBigUpdate())
        {
            const AI*    parent = m_ai->getParent()->getAI();
            const Block *target = parent->getTarget();

            if (target != pathTarget)
                path.IsFinished = true;

            if (path.IsFinished)
            {
                if (target)
                {
                    pathTarget = target;
                    path.setPathDest(target->getClusterPos(), 2.f * target->cluster->getCoreRadius());
                    status = "Pathing to Target";
                }
                else
                {
                    path.IsFinished = true;
                    return noAction("No Target");
                }
            }
        }

        return path.update(0);
    }
};


uint AInvestigate::update(uint blockedLanes)
{
    if (path.IsFinished && m_ai->isBigUpdate())
    {
        float2 investigatePos;
        float  investigateRadius = getWaypointRadius();

        if (m_ai->target)
        {
            // other actions that only work with line of sight may rely on this for path finding
            // they should resume control before we actually get all the way there
            investigatePos = m_ai->target->getClusterPos();
            investigateRadius = 2.f * m_ai->target->cluster->getCoreRadius();
            status = "Pathing to Target";
        }
        else if (m_ai->canEstimateTargetPos())
        {
            investigatePos = m_ai->estimateTargetPos();
            status = "Pathing to Estimated Target";
        }
        else if (m_ai->healTarget)
        {
            investigatePos = m_ai->healTarget->getAbsolutePos();
            investigateRadius = max(investigateRadius, m_ai->healRange);
            status = "Pathing to Heal Ally";
        }
        else if (m_ai->getQuery().queryObstacles(m_ai->command).size())
        {
            const vector<Obstacle>& obs = m_ai->getQuery().getLast();
            float2 avgObstPos;
            float2 avgObstVel;
            foreach (const Obstacle& ob, obs)
            {
                avgObstPos += ob.pos;
                avgObstVel += ob.vel;
            }
            avgObstPos /= obs.size();
            avgObstVel /= obs.size();

            if (nearZero(avgObstVel))
                investigatePos = avgObstPos;
            else
                intersectRayCircle(&investigatePos, avgObstPos, -avgObstVel,
                                   getClusterPos(), getCluster()->getSensorRadius());
            status = "Pathing to Obstacle";
        }
        else if (m_ai->priorityTarget)
        {
            const BlockCluster *pcl = m_ai->priorityTarget->cluster;
            investigatePos = pcl->getAbsolutePos();
            investigateRadius = 2.f * (pcl->getCoreRadius() + getCluster()->getSensorRadius());
            status = "Pathing to Priority Target";
        }

        if (nearZero(investigatePos))
        {
            path.IsFinished = true;
            return noAction("");
        }

        path.setPathDest(investigatePos, investigateRadius);
    }

    return path.update(0);
}

struct AStop final : public AIAction {

    AMove move;

    static bool supportsConfig(const AICommandConfig& cfg) { return AMove::supportsConfig(cfg); }

    AStop(AI* ai) : AIAction(ai, LANE_MOVEMENT), move(ai) { }

    virtual uint update(uint blockedLanes)
    {
        snConfigDims cfg;
        cfg.cfg.velocity = float2(0.f);
        cfg.dims = SN_VELOCITY;
        move.setMoveConfig(cfg, 0.f);
        return move.update(0);
    }
};

struct AJustGo final : public AIAction {

    AMove move;

    static bool supportsConfig(const AICommandConfig& cfg) { return AMove::supportsConfig(cfg); }

    AJustGo(AI* ai) : AIAction(ai, LANE_MOVEMENT), move(ai) { }

    virtual uint update(uint blockedLanes)
    {
        snConfigDims cfg;
        cfg.cfg.velocity = 10000.f * getCluster()->getRot();
        cfg.dims = SN_VELOCITY;
        move.setMoveConfig(cfg, 0.f);
        return move.update(0);
    }
};

bool AWander::supportsConfig(const AICommandConfig& cfg) { return AMove::supportsConfig(cfg); }
float AWander::getMoveRad() const { return 4 * getClusterBRadius(); }

uint AWander::update(uint blockedLanes)
{
    if (move.IsFinished || needsRepath())
    {
        float2 myPos       = getClusterPos();
        float  wanderRange = 20 * getClusterBRadius();
        float2 wanderDest;
        for (uint i=0; i<10; i++) {
            wanderDest = myPos + wanderRange * angleToVector(randangle());
            if (!isDestObstructed(wanderDest))
                break;
            wanderRange *= 0.8f; // after 10 iterations, will be 10% of range (2 radius)
        }
        move.setMoveDest(wanderDest, getMoveRad());
    }

    return move.update(0);
}

struct ARotate final : public AIAction {

    AMove move;
    float2     pos;

    static bool supportsConfig(const AICommandConfig& cfg) { return AMove::supportsConfig(cfg); }
    float getMoveRad() const { return 4 * getClusterBRadius(); }

    ARotate(AI* ai) : AIAction(ai, LANE_MOVEMENT), move(ai) { }

    virtual uint update(uint blockedLanes)
    {
        if (move.IsFinished)
        {
            if (nearZero(pos))
                pos = getClusterPos();
            move.setMoveDestAngle(pos, getClusterBRadius(), randangle(), M_PI/6.f);
        }

        return move.update(0);
    }
};


struct ACollectBase : public AIAction {

    float                           tractorRange = 0.f;
    watch_ptr<const ResourcePocket> seekRes;


    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return AMove::supportsConfig(cfg) &&
            cfg.usesResources();
    }

    virtual void render(void* lineVoid) const
    {
        VertexPusherLine& line = *(VertexPusherLine*)lineVoid;
        if (exist(seekRes))
        {
            line.color(COLOR_RESOURCE0);
            line.PushCircle(seekRes->getPos(), tractorRange);
            line.PushLine(getClusterPos(), seekRes->getPos());
        }
    }

    // return true if we should collect
    bool checkTractorCapacity(float capacity)
    {
        const BlockCluster *cluster = getCluster();
        if (!(cluster->getFeatureUnion()&Block::TRACTOR))
        {
            status = "No Tractor";
            return false;
        }

        // only collect if we have room for resources
        if (capacity - cluster->getResources() < kMinResPocketQuant)
        {
            status = "Capacity Filled";
            return false;
        }

        if (seekRes && seekRes->owner)
        {
            seekRes = NULL;
        }

        tractorRange = max(cluster->getBRadius(),
                           cluster->getTractorBRadius());

        return true;
    }

    void setSeekRes(const ResourcePocket* rp)
    {
        seekRes   = rp;
        m_ai->defendPos = (getClusterPos() + rp->getPos()) / 2.f;
    }

    ACollectBase(AI* ai, uint lanes=LANE_MOVEMENT) : AIAction(ai, lanes) {}
};

// pick up large resource pockets
struct ACollect final : public ACollectBase {

    APath      path;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return APath::supportsConfig(cfg) &&
            cfg.usesResources();
    }

    ACollect(AI* ai) : ACollectBase(ai), path(ai) {}

    virtual void render(void* line) const
    {
        path.render(line);
        ACollectBase::render(line);
    }

    virtual uint update(uint blockedLanes)
    {
        if (!checkTractorCapacity(getCluster()->getResourceCapacity()))
            return LANE_NONE;

        if (m_ai->isBigUpdate() && (path.IsFinished || !seekRes))
        {
            vector<ResourcePocket*>& resources = m_ai->getVisibleResources();

            float maxRes    = 5.f * kMinResPocketQuant; // only interested in large pockets
            path.IsFinished = true;

            foreach (const ResourcePocket* rp, resources)
            {
                // the resources we would get if we were at that resource's pos
                // (trying to detect clusters of small resources)
                float selectQuant = rp->quantity;
                foreach (const ResourcePocket* rp1, resources) {
                    if (rp1 != rp && intersectPointCircle(rp1->getPos(), rp->getPos(), tractorRange))
                        selectQuant += rp1->quantity;
                }

                if (selectQuant > maxRes) {
                    maxRes = selectQuant;
                    setSeekRes(rp);
                    path.setPathDest(seekRes->getPos(), tractorRange);
                }
            }

            status = (path.IsFinished) ? "No !/$ Resource" : "Collecting";
        }

        return path.update(0);
    }
};


// wander around picking up resources
struct ACollectWander final : public ACollectBase {

    AMove move;
    float yieldThreshold;

    ACollectWander(AI* ai, float yt) : ACollectBase(ai), move(ai), yieldThreshold(yt) {}

    virtual uint update(uint blockedLanes)
    {
        const BlockCluster* cluster = getCluster();
        const float capacity = (yieldThreshold > 0 && cluster->getBlueprint())
                               ? min(cluster->getResourceCapacity(),
                                     cluster->getBlueprint()->getBudResource() * yieldThreshold) :
                               cluster->getResourceCapacity();
        if (!checkTractorCapacity(capacity))
            return LANE_NONE;

        if ((m_ai->isBigUpdate() && (move.IsFinished || !seekRes)) || needsRepath())
        {
            const float2 myPos = cluster->getAbsolutePos();

            vector<ResourcePocket*>& resources = m_ai->getVisibleResources();
            vec_sort_key(resources, [&](const ResourcePocket *rp) {
                return -(rp->quantity / distance(rp->getPos(), myPos)); });

            seekRes = NULL;
            foreach (const ResourcePocket *rp, resources)
            {
                if (!intersectPointCircle(rp->getPos(), cluster->getAbsolutePos(), tractorRange) &&
                    !isDestObstructed(rp->getPos()))
                {
                    setSeekRes(rp);
                    break;
                }
            }

            if (seekRes)
            {
                status = "Collecting";
                move.setMoveDest(seekRes->getPos(), tractorRange);
            }
        }

        if (!seekRes)
            return noAction("No Resources");

        return move.update(0);
    }
};

struct AClearDebris final : public AIAction {

    AMove                  move;
    watch_ptr<const Block> scavengeBlock;
    ScavengeCaps           caps;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return AMove::supportsConfig(cfg) && cfg.hasWeapons;
    }

    AClearDebris(AI* ai) : AIAction(ai, LANE_MOVEMENT|LANE_SHOOT|LANE_TARGET), move(ai) {}

    virtual uint update(uint blockedLanes)
    {
        const BlockCluster *cluster = getCluster();

        if (!caps.initialized)
            caps = getScavengeCaps(cluster);

        if (caps.maxRange == 0.f || caps.bestEfficiency == 0.f)
            return noAction("No Weapons");

        if (m_ai->isBigUpdate())
        {
            BlockPattern pat = patternForAssemble(cluster, 0);
            pat.radius = caps.maxRange;
            pat.antifeatures |= Block::ENVIRONMENTAL|Block::SEED|Block::UNIQUE|Block::UNGROW|Block::PERISHABLE;
            scavengeBlock = cluster->zone->intersectCircleBlocksNearest(pat);
            if (!scavengeBlock)
                return noAction("No Blocks");
        }

        // this can happen if a faction 0 ship assembles the block after targetting...
        if (scavengeBlock && scavengeBlock->cluster == cluster)
            scavengeBlock = NULL;

        if (!scavengeBlock)
            return LANE_NONE;

        status = "Destroying";
        FiringData data(scavengeBlock);
        data.filter = ~(Block::LAUNCH|Block::LAUNCHER);
        data.filter.minEfficiency = caps.bestEfficiency;
        m_ai->fireWeaponsAt(data);
        if (caps.bestIsFixed) {
            move.setMoveRot(directionForFixed(cluster, data), 0.1);
            move.update(0);
        }
        return LANE_SHOOT|LANE_MOVEMENT|LANE_TARGET;
    }

    virtual void render(void* lineVoid) const
    {
        VertexPusherLine& line = *(VertexPusherLine*)lineVoid;
        const Block* bl = scavengeBlock.get();
        Block::lock_guard l(bl);
        if (!l.valid())
            return;
        line.color(COLOR_ENEMY_HI2);
        line.PushCircle(bl->getAbsolutePos(), bl->spec->radius);
        line.PushCircle(bl->cluster->getAbsolutePos(), bl->cluster->getBRadius());
        line.PushLine(getClusterPos(), bl->getAbsolutePos());
    }

};

bool AScavengeWeapon::supportsConfig(const AICommandConfig& cfg)
{
    return (cfg.features&Block::ASSEMBLER);
}

uint AScavengeWeapon::update(uint blockedLanes)
{
    BlockCluster *cluster = m_ai->command->cluster;
    Block *assembler = cluster->getAssembler();

    if (!assembler || assembler->getAssemblerConnections())
        return noAction("Waiting for Assembly");

    if (!m_ai->isBigUpdate())
        return LANE_NONE;

    if (cluster->countTransients() >= kAIMaxTransients)
        return noAction("Full");

    BlockPattern pat = patternForAssemble(cluster, 0);
    pat.antifeatures |= Block::SEED|Block::PERISHABLE;
    BlockList blocks;
    cluster->zone->intersectCircleBlocks(&blocks, pat);

    Block* bestTotal    = NULL;
    int    bestTotalRes = 1;

    foreach (Block* bl, blocks)
    {
        int res = 0;
        if (bl->sb.features == 0 ||
            bl->cluster == cluster ||
            (bl->cluster->getFeatureUnion()&Block::PERSISTENT) ||
            (bl->getMass() >= cluster->getMass() / 2.f) ||
            ((res = bl->sb.deadliness()) <= bestTotalRes) ||
            (bl->sb.getFeaturesFromId()&Block::SEED) ||
            fitFailed.count(bl->sb.spec().sid))
            continue;
        ASSERT(bl->sb.ident != 500);
        bestTotal = bl;
        bestTotalRes = res;
    }

    if (!bestTotal)
        return noAction("No Blocks");

    if (!cluster->attachTransient(bestTotal))
    {
        fitFailed.insert(bestTotal->sb.spec().sid);
        return noAction("Connect failed");
    }

    status = "Tractoring";
    return LANE_ASSEMBLE;
}

struct AKillPlants : public ACollectBase {

    AMove             move;
    watch_ptr<const Block> targetSeed;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return APath::supportsConfig(cfg) &&
            cfg.hasWeapons &&
            cfg.usesResources();
    }

    AKillPlants(AI* ai) : ACollectBase(ai, LANE_MOVEMENT|LANE_SHOOT|LANE_TARGET), move(ai) {}

    virtual uint update(uint blockedLanes)
    {
        const BlockCluster *cluster = getCluster();

        if (!checkTractorCapacity(cluster->getResourceCapacity()))
            return false;

        const ScavengeCaps caps = getScavengeCaps(cluster);
        if (caps.maxRange == 0.f || caps.bestEfficiency == 0.f)
            return noAction("No Weapons");

        if (m_ai->isBigUpdate())
        {
            BlockPattern pat = patternForKillPlants(m_ai->command);
            targetSeed = cluster->zone->intersectCircleBlocksNearest(pat);
        }

        if (!targetSeed)
            return noAction("No Plants");

        const float range = min(tractorRange, caps.maxRange);
        if (!intersectPointCircle(cluster->getAbsolutePos(), targetSeed->getClusterPos(), range))
            return noAction("Plant too far away");

        status = "Destroying";
        FiringData data(targetSeed);
        m_ai->defendPos = data.pos;
        data.filter.minEfficiency = caps.bestEfficiency;
        const int count = m_ai->fireWeaponsAt(data);
        if (caps.bestIsFixed) {
            move.setMoveRot(directionForFixed(cluster, data), 0.1);
            move.update(0);
            status = "Destroying/Rotating";
        }
        return (count ? (LANE_SHOOT|LANE_TARGET) : 0)|LANE_MOVEMENT;
    }

    virtual void render(void* lineVoid) const
    {
        VertexPusherLine& line = *(VertexPusherLine*)lineVoid;
        const Block* bl = targetSeed.get();
        if (exist(bl))
        {
            line.color(COLOR_ENEMY_HI2);
            line.PushCircle(bl->getAbsolutePos(), bl->spec->radius);
            line.PushCircle(bl->cluster->getAbsolutePos(), bl->cluster->getBRadius());
            line.PushLine(getClusterPos(), bl->getAbsolutePos());
        }
    }

};

template <typename T, typename Comp>
T selectMinRandomized(vector<T>& vec, size_t mx, const Comp& comp)
{
    size_t count = min(mx, vec.size());
    ASSERT(vec.size());
    if (count == 1)
        return vec[0];
    vec_selection_sort(vec, count, comp);
    const size_t idx = rand_positive_unormal(count, count / 4.f);
    ASSERT(idx < count);
    return vec[idx];
}

namespace Planting {

    const float kMinPorts = 3;

    // select a nearby block that would be good for planting
    // randomized, with a bias towards more suitable blocks
    static const Block* findDirtBlock(const BlockCluster* cluster)
    {
        static const uint kMaxSelectionBlocks = 30;

        BlockList blocks;
        cluster->zone->intersectCircleBlocks(&blocks, patternForPlanting(cluster));

        for (uint i=0; i<blocks.size(); )
        {
            const uint freePorts = blocks[i]->countFreePorts();
            if (freePorts < kMinPorts) { // don't even bother
                vec_pop(blocks, i);
            } else  {
                blocks[i]->compareKey = 0x1000000 *  blocks[i]->countConnectedPorts(Block::SEED) +
                                        0x10000   * (1000 - freePorts) +
                                        distanceSqr(blocks[i]->getAbsolutePos(), cluster->getAbsolutePos());
                i++;
            }
        }

        if (blocks.size() == 0)
            return NULL;

        return selectMinRandomized(blocks, kMaxSelectionBlocks, Block::lessByKey);
    }

    static float2 getAvgPortNormal(const Block* bl)
    {
        float2 norm;
        for (uint i=0; i<bl->spec->portCount; i++) {
            if (bl->isPortFree(i))
                norm += bl->getPortAbsoluteNormal(i);
        }
        return normalize_orzero(norm);
    }

};

struct APlantSelf final : public AIAction {

    AMove             move;
    APath             path;
    int                    tries  = 0;
    watch_ptr<const Block> dirtBlock;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return APath::supportsConfig(cfg) && cfg.isRoot();
    }

    APlantSelf(AI* ai) : AIAction(ai, LANE_MOVEMENT|LANE_SHOOT|LANE_TARGET), move(ai), path(ai) {}

    virtual uint update(uint blockedLanes)
    {
        const BlockCluster *cluster = getCluster();

        if (m_ai->isBigUpdate())
        {
            if (!dirtBlock)
                dirtBlock = Planting::findDirtBlock(cluster);
            if (!dirtBlock)
                return noAction("No Dirt");

            if (!needsRepath())
            {
                const float  seedRad = cluster->getBRadius();
                const float2 seedPos = cluster->getAbsolutePos();
                const float2 destPos = dirtBlock ? dirtBlock->getAbsolutePos() :
                                       seedPos + (length(cluster->getVel()) > 100.f ?
                                                  (2.f * cluster->getVel()) :
                                                  (500.f * cluster->getRot()));
                const Block* obs = cluster->zone->intersectSegmentBlocks(SegmentPattern(seedPos, destPos, 0.5f * seedRad,
                                                                                        cluster, 0, Block::ENVIRONMENTAL)).block;
                if (obs && obs->countFreePorts() >= Planting::kMinPorts)
                {
                    status = "Ramming Dirt";
                    snConfigDims cfg;
                    obs->getNavConfig(&cfg.cfg);
                    cfg.cfg.angle = vectorToAngle(cluster->getAbsolutePos() - obs->getAbsolutePos());
                    cfg.dims = SN_POSITION|SN_TARGET_VEL|SN_ANGLE;
                    move.setMoveConfig(cfg, obs->spec->minradius + cluster->getBRadius());
                    path.IsFinished = true;

                    return move.update(0)|LANE_ASSEMBLE;
                }
            }
        }

        if (dirtBlock && path.IsFinished && m_ai->isBigUpdate())
        {
            status = "Pathing to Dirt";

            const float  seedRad = cluster->getBRadius();
            const float2 dirtPos = dirtBlock->getAbsolutePos();
            const float2 dest    = dirtPos + (2.f * (seedRad + dirtBlock->spec->radius) *
                                              Planting::getAvgPortNormal(dirtBlock.get()));
            move.IsFinished = true;
            path.setPathDest(dest, 2.f * seedRad);
        }

        return move.update(0) ? (LANE_MOVEMENT|LANE_ASSEMBLE) : path.update(0);
    }

    virtual void render(void* lineVoid) const
    {
        VertexPusherLine& line = *(VertexPusherLine*)lineVoid;
        if (dirtBlock)
        {
            line.color(COLOR_GREEN);
            line.PushCircle(dirtBlock->getAbsolutePos(), dirtBlock->spec->radius);
        }
        if (move.IsFinished)
            path.render(lineVoid);
    }

};

struct APlant final : public AIAction {

    AMove             move;
    APath             path;
    watch_ptr<const Block> dirtBlock;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return APath::supportsConfig(cfg) && (cfg.features&Block::LAUNCH);
    }

    APlant(AI* ai) : AIAction(ai, LANE_MOVEMENT|LANE_SHOOT|LANE_TARGET), move(ai), path(ai) {}

    static bool getSeedLaunch(const BlockCluster *cl, Block** seedLauncher, Block** seedLaunch)
    {
        if (!(cl->getFeatureUnion()&Block::LAUNCHER))
            return false;
        bool hasLauncher = false;
        foreach (Block* bl, cl->blocks)
        {
            if ((bl->sb.features&Block::LAUNCHER) && (bl->sb.replicateBlock->features&Block::SEED)) {
                if (seedLauncher)
                    *seedLauncher = bl;
                hasLauncher = true;
            }
            if ((bl->sb.features&Block::LAUNCH) && (bl->sb.launchFeatures&Block::SEED)) {
                if (seedLaunch)
                    *seedLaunch = bl;
            }
            if ((seedLaunch && seedLauncher) ?  (*seedLaunch && *seedLauncher) : hasLauncher)
                break;
        }
        return hasLauncher;
    }

    virtual uint update(uint blockedLanes)
    {
        const BlockCluster *cluster = getCluster();
        ASSERT(m_ai->command == cluster->command);

        Block* seedLauncher = NULL;
        Block* seedLaunch   = NULL;
        if (!getSeedLaunch(cluster, &seedLauncher, &seedLaunch))
            return noAction("No Seed Launcher");
        if (seedLaunch && !seedLaunch->isLaunchEnoughRes())
            return noAction("Not enough res to launch");

        if (m_ai->isBigUpdate())
        {
            if (seedLaunch)
            {
                const float2 seedPos = seedLaunch->getAbsolutePos();
                const float2 seedVel = seedLaunch->getLaunchVel();
                const Block* obs     = cluster->zone->intersectSegmentBlocks(
                    SegmentPattern(seedPos, seedPos + 2.f * seedVel, 0.5f * seedLaunch->spec->radius,
                                   cluster, 0, Block::ENVIRONMENTAL)).block;

                if (obs)
                {
                    seedLaunch->setEnabled(Block::LAUNCH);
                    //move.IsFinished = true;
                    //path.IsFinished = true;
                    status = "Launching Seed";
                    return LANE_SHOOT;
                }
            }

            if (path.IsFinished && move.IsFinished)
            {
                dirtBlock = Planting::findDirtBlock(cluster);

                if (!dirtBlock)
                    return noAction("No Dirt");

                const float  myRad   = cluster->getBRadius();
                const float2 dirtPos = dirtBlock->getAbsolutePos();
                const float2 dest    = dirtPos + (2.f * (myRad + dirtBlock->spec->radius) *
                                                  Planting::getAvgPortNormal(dirtBlock.get()));
                if (path.isAtDest())
                {
                    status = "Aiming Seed";
                    const Block* seed = seedLaunch ? seedLaunch : seedLauncher;
                    move.setMoveAngle(seed->weaponAngleForTarget(dirtPos, dirtBlock->cluster->getVel()), 0.1);
                }
                else
                {
                    status = "Pathing to Dirt";
                    path.setPathDest(dest, dirtBlock->spec->radius * 2.f);
                }
            }
        }

        if (move.update(0) == LANE_MOVEMENT)
            return LANE_MOVEMENT;
        return path.update(0);
    }

    virtual void render(void* lineVoid) const
    {
        VertexPusherLine& line = *(VertexPusherLine*)lineVoid;
        if (dirtBlock)
        {
            line.color(COLOR_GREEN);
            line.PushCircle(dirtBlock->getAbsolutePos(), dirtBlock->spec->radius);
        }
        if (move.IsFinished)
            path.render(lineVoid);
    }

};



struct AResToParent final : public AIAction {

    APath  path;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return cfg.hasParent && AMove::supportsConfig(cfg) && (cfg.features&Block::TRACTOR);
    }

    AResToParent(AI* ai) : AIAction(ai, LANE_MOVEMENT), path(ai) { }

    virtual uint update(uint blockedLanes)
    {
        const Block*        parent = m_ai->getParent();
        const BlockCluster *pcl    = parent ? parent->cluster : NULL;

        if (!parent || !(pcl->getFeatureUnion()&Block::TRACTOR))
            return noAction("No Parent with tractor");
        const float parentCapacity = pcl->getResourceCapacity() - pcl->getResources();
        if (parentCapacity < kMinResPocketQuant)
            return noAction("Parent is full");

        const BlockCluster *mycl = getCluster();
        if (mycl->getResources() < kMinResPocketQuant ||
            mycl->getResources() < 0.1f * mycl->getResourceCapacity())
        {
            return noAction("Too few resources");
        }

        const float returnDist = pcl->getTractorBRadius();
        const float2 returnPos = pcl->getAbsolutePos() + 1.5f * pcl->getVel();

        if (path.IsFinished || !intersectPointCircle(returnPos, path.pos, path.rad)) {
            path.setPathDest(returnPos, returnDist);
        }

        status = "Returning";
        return path.update(0);
    }

    virtual void render(void* line) const { path.render(line); }

};


struct AReturnParent final : public AIAction {

    APath path;
    float2     destp;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return APath::supportsConfig(cfg);
    }

    AReturnParent(AI* ai) : AIAction(ai, LANE_MOVEMENT), path(ai) { }

    virtual uint update(uint blockedLanes)
    {
        Block* parent = m_ai->getParent();
        if (!parent)
            return noAction("No Parent");
        const BlockCluster *pc         = parent->cluster;
        const BlockCluster *myc        = getCluster();
        const float         returnDist = max(kAIActionFollowDistance,
                                             2.f * (myc->getBRadius() + pc->getBRadius()));
        const float2 ppos    = pc->getAbsolutePos();
        const float2 mypos   = myc->getAbsolutePos();
        const float  pathrad = returnDist / 2.f;

        destp = ppos + normalize(mypos - ppos) * returnDist;

        if (intersectPointCircle(mypos, destp, pathrad))
            return noAction("close enough");

        if (path.IsFinished || !intersectPointCircle(path.pos, destp, pathrad))
            path.setPathDest(destp, pathrad);
        m_ai->defendPos = ppos;

        status = "Pathing";
        return path.update(0);
    }

    virtual void render(void* line) const { path.render(line); }
};


// follow parent around, including pathing if we get too far away
struct AFollowParent final : public AIAction {

    AReturnParent ret;
    AMove         move;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return APath::supportsConfig(cfg);
    }

    AFollowParent(AI* ai) : AIAction(ai, LANE_MOVEMENT), ret(ai), move(ai) { }

    virtual uint update(uint blockedLanes)
    {
        Block* parent = m_ai->getParent();
        if (!parent)
            return noAction("No Parent");
        const uint lanes = ret.update(0);
        if (lanes != LANE_NONE)
            return lanes;

        const BlockCluster *pc  = parent->cluster;
        const BlockCluster *myc = getCluster();

        ////////////////////////////////////////////////////////////////
        /////////////////////   FIXME   ////////////////////////////////
        ////////////////////////////////////////////////////////////////

        snConfigDims dest;
        dest.cfg.angle    = pc->getAngle();
        dest.cfg.velocity = pc->getVel();
        dest.cfg.position = ret.destp;
        dest.dims         = SN_ANGLE|SN_POSITION|SN_VELOCITY;

        move.setMoveConfig(dest, myc->getBRadius());

        status = "Aligning";
        return move.update(0);
    }

    virtual void render(void* line) const { ret.render(line); move.render(line); }
};

struct AFollowAllies final : public AIAction {

    AMove  move;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return AMove::supportsConfig(cfg);
    }

    AFollowAllies(AI* ai) : AIAction(ai, LANE_MOVEMENT), move(ai) { }

    virtual uint update(uint blockedLanes)
    {
        const BlockList& allies = m_ai->getAllies();

        const float farDist = max(0.25f * getCluster()->getSensorRadius(),
                                  getWaypointRadius());
        const float2 clpos = getClusterPos();
        const float clrad = getClusterBRadius();

        bool   isfar    = true;
        int    aallies = 0;
        float2 apos;
        float2 avel;
        float2 adir;
        foreach (const Block* bl, allies)
        {
            if (!bl->cluster->isMobile() || bl->cluster == getCluster())
                continue;
            const float2 blpos = bl->cluster->getAbsolutePos();
            if (intersectCircleCircle(blpos, bl->cluster->getBRadius(), clpos, clrad + farDist))
            {
                isfar = false;
            }
            apos += blpos;
            avel += bl->cluster->getVel();
            adir += bl->cluster->getRot();
            aallies++;
        }

        if (!aallies)
            return noAction("No Mobile Allies");

        snConfigDims dest;
        dest.cfg.angle    = vectorToAngle(adir);
        dest.cfg.velocity = avel / (float) aallies;
        dest.dims         = SN_ANGLE|SN_VELOCITY;

        if (isfar) {
            dest.cfg.position = apos / (float) allies.size();
            dest.dims |= SN_POSITION;
            status = "Moving Closer";
        } else {
            status = "Aligning";
        }

        move.setMoveConfig(dest, getClusterBRadius());

        return move.update(0);
    }

    virtual void render(void* line) const { move.render(line); }
};

bool AISignal::satisfiesNeed(const BlockCluster *cl) const
{
    switch (type)
    {
    case NONE:       return false;
    case DEADLINESS: return true;
    case CAPACITY:   return cl->getResourceCapacity() - cl->getResources() > kMinResPocketQuant;
    }
    return false;
}

float AISignal::satisfiedNeed() const
{
    if (type == NONE)
        return FLT_MAX;
    float satNeed = 0.f;
    foreach (const watch_ptr<const Block>& bl, responders)
    {
        if (!exist(bl))
            continue;
        switch (type)
        {
        case NONE:       break;
        case DEADLINESS: satNeed += bl->cluster->getDeadliness(); break;
        case CAPACITY:   satNeed += bl->cluster->getResourceCapacity() - bl->cluster->getResources(); break;
        }
    }
    return satNeed;
}

struct ARespondToSignal final : public AIAction {

    APath             path;
    uint                   types;
    const char*            status;
    watch_ptr<const Block> respondSignalOwner;


    virtual void render(void* line) const { path.render(line); }
    static bool supportsConfig(const AICommandConfig& cfg) { return APath::supportsConfig(cfg); }

    virtual string toStringEx() const
    {
        return (respondSignalOwner && respondSignalOwner->commandAI) ?
            "Responding to: " + respondSignalOwner->commandAI->signal.toString() :
            (!path.IsFinished) ? "Remembered signal" : "No Signal";
    }

    ARespondToSignal(AI* ai, uint tps) : AIAction(ai, LANE_MOVEMENT), path(ai), types(tps) { }

    const Block*    signalOwner;
    uint            minPriority;
    float           minDist;

    void findSignal(const Block* bl)
    {
        if (!bl || !bl->commandAI)
            return;

        const AISignal& sig  = bl->commandAI->signal;
        const float     dist = distance(sig.pos, getClusterPos());
        if ((sig.type&types) &&
            ((sig.priority < minPriority) || (sig.priority == minPriority && dist < minDist)) &&
            sig.satisfiesNeed(getCluster()) &&
            sig.hasNeed())
        {
            minPriority = sig.priority;
            minDist     = dist;
            signalOwner = bl;
        }
    }

    virtual uint update(uint blockedLanes)
    {
        if (respondSignalOwner && (!respondSignalOwner->commandAI ||
                                   respondSignalOwner->commandAI->signal.type == AISignal::NONE))
        {
            respondSignalOwner = NULL;
        }
        vec_pop_if_not(m_ai->signal.responders);

        minPriority = std::numeric_limits<uint>::max();
        minDist     = FLT_MAX;
        signalOwner = NULL;

        foreach (const Block* bl, m_ai->getAllies())
        {
            findSignal(bl);
        }

        if (path.IsFinished || (signalOwner != respondSignalOwner.get()))
        {
            if (respondSignalOwner) {
                vec_remove_one(respondSignalOwner->commandAI->signal.responders,
                               m_ai->command);
            }
            respondSignalOwner = signalOwner;

            if (respondSignalOwner)
            {
                AISignal &sig = respondSignalOwner->commandAI->signal;
                vec_add(sig.responders, make_watch(m_ai->command));
                path.setPathDest(sig.pos, sig.radius);
            }
            // if we lost the signal and no other signals, go and check it out anyway
        }

        return path.update(0);
    }
};

struct APatrolParent final : public AIAction {

    APath path;

    virtual void render(void* line) const { path.render(line); }
    static bool supportsConfig(const AICommandConfig& cfg) { return APath::supportsConfig(cfg); }

    APatrolParent(AI* ai) : AIAction(ai, LANE_MOVEMENT),
                                 path(ai) { }

    virtual uint update(uint blockedLanes)
    {
        Block* parent = m_ai->getParent();
        if (!parent)
            return LANE_NONE;
        const BlockCluster *parentCluster = parent->cluster;
        const BlockCluster *cluster       = getCluster();

        // FIXME needs better handling for moving parents

        if (path.IsFinished)
        {
            const float2 myPos     = cluster->getAbsolutePos();
            const float2 parentPos = parentCluster->getAbsolutePos();

            const float sensorRad  = cluster->getSensorRadius();
            const float patrolRad = parentCluster->getBRadius() + sensorRad;
            if (intersectCircleCircle(myPos, cluster->getBRadius(), parentPos, patrolRad))
            {
                // if we are within the patrol radius, try to space out from nearby allies while
                // remaining within the patrol radius
                const float  spaceRad = max(2.f * cluster->getBRadius(), 0.5f * sensorRad);
                BlockList nearest;
                cluster->zone->intersectCircleBlocks(&nearest, patternForAllies(cluster, spaceRad));

                // if nothing is too nearby, maintain our position relative to parent
                if (nearest.empty()) {
                    snConfig cfg;
                    cfg.velocity = parent->cluster->getVel();
                    cfg.angle    = parent->cluster->getAngle();
                    m_ai->nav->setDest(cfg, SN_VELOCITY|SN_ANGLE, 0);
                    return LANE_MOVEMENT;
                }

                const float2 nearestCenter = vec_average<float2>(nearest, [](const Block *bl) {
                    return bl->cluster->getAbsolutePos(); });
                const float  nearestRadius = sqrt(nearest.size()) * spaceRad;

                float2 ra, rb;
                int points = intersectCircleCircle(&ra, &rb, parentPos, patrolRad, nearestCenter,
                                                   nearestRadius);
                float2 dest;
                if (points != 2) {
                    // spacing around nearby clusters does not intersect with patrol radius
                    // fly to perimeter of clusters
                    dest = nearestCenter + nearestRadius * normalize(myPos - nearestCenter);
                } else {
                    // fly to nearest point on perimeter that is outside of spacing radius
                    dest = (distance(ra, myPos) < distance(rb, myPos)) ? ra : rb;
                }
                path.setPathDest(dest, cluster->getBRadius());
            }
            else
            {
                // if we are far away, return to patrol radius
                path.setPathDest(parentPos, patrolRad);
            }
        }

        return path.update(0);
    }

};

bool ABudReproduce::supportsConfig(const AICommandConfig& cfg)
{
    return (cfg.features&Block::ASSEMBLER) && (cfg.features&(Block::FACTORY|Block::SELFFACTORY|Block::TELESPAWN));
}

void ABudReproduce::onReset()
{
    if (!(m_ai->sc->flags&SerialCommand::CHILDREN_SET))
        m_ai->sc->children.clear();
}

uint ABudReproduce::update(uint blockedLanes)
{
    const BlockCluster *cluster = getCluster();
    const GameZone *zone = m_ai->zone;

    // reeaaaly no need to run this frequently
    if (!m_ai->isSuperUpdate())
        return LANE_NONE;

    SerialCommand *sc = m_ai->sc;
    if (!sc)
        return LANE_NONE;

    if (!(sc->flags&SerialCommand::ATTACK))
    {
        if ((zone->simTime - m_ai->lastDamagedTime) < kAIDamageReproduceCooldown)
            return noAction("Under fire");
    }

    const float wantFleetP = kAIParentPFleetRatio * cluster->getDeadliness();
    float fleetP = 0.f;

    followers.clear();
    foreach (Block* bl, m_ai->getAllies())
    {
        const AI *ai = bl->getCommandAI();
        if (ai && ai->getParent() == m_ai->command)
        {
            fleetP += bl->cluster->getDeadliness();
            followers.push_back(bl);
        }
    }

    // release followers if there are too many
    while (fleetP > wantFleetP && followers.size() > 1)
    {
        Block *bl = randselect_pop(followers);
        m_ai->removeChild(bl);
    }

    if (zone->getCommandsLimitRemaining(getClusterPos()) <= 0)
        return noAction("Sector Limit Reached");

    const Feature_t features = m_ai->getConfig().features;

    if (!vec_any(sc->children))
    {
        const BlockCluster *bp1 = m_ai->command->getBlueprint();
        if (!bp1)
            return LANE_NONE;
        const float totaldl = bp1->getDeadliness();
        BlueprintList childs;
        if (features&Block::FACTORY) {
            float dl = cluster->hasFreeRes() ? randrange(totaldl/2.f, totaldl) :
                       cluster->getResources();
            Fleetspec spec(m_ai->getFaction(), dl, 3);
            spec.flags = EFleetspecFlags::PYRAMID|EFleetspecFlags::SELECTION;
            if (!cluster->isMobile())
                spec.flags |= EFleetspecFlags::MOBILE;
            childs = spec.compose(zone->save);
        } else if (features&Block::SELFFACTORY) {
            childs.push_back(selectShip(zone->save, m_ai->getFaction(), floor_int(0.5f * totaldl), ceil_int(1.5f * totaldl)));
        } else if (features&Block::TELESPAWN) {
            foreach (const watch_ptr<Block> &bl, m_ai->getChildren())
            {
                if (bl && !bl->sb.isTransient() && bl->sb.lifetime <= 0.f) {
                    status = "Child Exists";
                    return LANE_NONE;
                }
            }
            const int points = m_ai->command->sb.command->energy;
            childs.push_back(selectShip(zone->save, m_ai->getFaction(), points / 2, points));
        } else {
            status = "No factory";
            return LANE_NONE;
        }

        // stop spawning station cores!
        vec_pop_if(childs, [](const BlockCluster *bp) { return !bp || (bp->getFeatureUnion()&Block::ROOT); });

        if (childs.empty()) {
            status = "No ships to build";
            return LANE_NONE;
        }

        if ((features&Block::FACTORY) && fleetP < wantFleetP)
        {
            // adopt a child if possible
            foreach (Block* bl, m_ai->getAllies())
            {
                if (bl != m_ai->command &&
                    !bl->getAI()->getParent() &&
                    vec_any(childs, [&](const BlockCluster *cl) { return cl->hash == bl->cluster->hash; }) &&
                    !globals.isOnPlayer(bl) &&
                    bl->cluster->getDeadliness() + fleetP < wantFleetP)
                {
                    m_ai->adoptChild(bl);
                    status = "Adopted";
                    return LANE_ASSEMBLE;
                }
            }
        }

        // figure out which blueprint to bud
        sc->children.clear();
        foreach (const BlockCluster *bp, childs) {
            sc->children.push_back(BlueprintPtr(bp));
        }
        sc->currentChild = randrange(sc->children.size());
    }

    if (!vec_any(sc->children))
        return noAction("No child blueprints");

    const int           index   = vec_next(sc->children, sc->currentChild, 0);
    const BlockCluster* childBp = (index >= 0) ? sc->children[index].get() : NULL;

    if (!childBp)
        return noAction("Failed to get blueprint");

    if (m_ai->sc->flags&SerialCommand::MUTATES)
    {
        BlockCluster *cl = childBp->clone();
        cl->applyMutation();
        childBp = cl;
    }

    if (!cluster->hasFreeRes() && !(features&Block::TELESPAWN))
    {
        needRes = min(cluster->getResourceCapacity() - kMinResPocketQuant, childBp->getBudResource());
        const float haveRes = m_ai->command->getResources();
        if (needRes > haveRes) {
            status = "Not enough resources";
            return LANE_NONE;
        }
    }

    Block* child = m_ai->command->cluster->budChild(childBp);
    if (child) {
        status = "Budded";
        m_ai->adoptChild(child);

        if (sc)
            sc->currentChild = vec_next(sc->children, sc->currentChild, +1);
    } else {
        status = "Bud failed";
    }
    return (child) ? LANE_ASSEMBLE : LANE_NONE;
}

bool AHeal::supportsConfig(const AICommandConfig& cfg)
{
    return cfg.features&(Block::ASSEMBLER|Block::REGROWER);
}

uint AHeal::update(uint blockedLanes)
{
    Block* command = m_ai->command->cluster->getRegrower();
    if (!exist(command) || !m_ai->sc->blueprint)
        return noAction("No Assembler or Blueprint");

    if (m_ai->isBigUpdate()) {
        matches = command->cluster->matchesBlueprint();
        valid = command->cluster->isBlueprintValid();
    }

    if (!valid)
        return noAction("invalid blueprint");
    else if (matches)
        return noAction("matches");

    status = "Assembling";
    command->setEnabled(Block::ASSEMBLER);
    return LANE_ASSEMBLE;
}

bool AMetamorphosis::supportsConfig(const AICommandConfig& cfg)
{
    return (cfg.features&(Block::ASSEMBLER|Block::REGROWER));
}

void AMetamorphosis::onReset()
{
    if (!(m_ai->sc->flags&SerialCommand::BLUEPRINT_SET))
        m_ai->sc->nextprint = NULL;
}

uint AMetamorphosis::update(uint blockedLanes)
{
    Block*         command = m_ai->command->cluster->getRegrower();
    SerialCommand *sc      = m_ai->sc;
    GameZone      *zone    = m_ai->zone;

    if (!exist(command) || !sc)
        return noAction("no regrower");

    if (!m_ai->isSuperUpdate())
        return LANE_NONE;

    if (!(sc->flags&SerialCommand::ATTACK))
    {
        if (sc->nextprint && sc->blueprint && sc->nextprint->hash == sc->blueprint->hash)
            return noAction("nextprint matches blueprint");

        if (m_ai->mood != AI::NEUTRAL || m_ai->target || m_ai->canEstimateTargetPos())
            return noAction("fighting");

        if ((zone->simTime - m_ai->lastDamagedTime) < kAIDamageReproduceCooldown)
            return noAction("Under fire");

        if (m_ai->getEnemyAllyRatio() > 0.75f)
            return noAction("Too many enemies");

        if (!(sc->flags&SerialCommand::BLUEPRINT_SET) &&
            zone->simTime - getCluster()->live->lastInitTime < kAIInitReproduceCooldown)
        {
            return noAction("Just inited");
        }
    }

    // choose cluster to metamorphose to once
    if (!sc->nextprint)
    {
        if ((m_ai->sc->flags&SerialCommand::MUTATES) && sc->blueprint)
        {
            BlockCluster *cl = sc->blueprint->clone();
            cl->applyMutation();
            sc->nextprint.reset(cl);
        }
        else
        {
            const BlockCluster *mycl  = getCluster();
            const int fac = m_ai->getFaction();
            BlueprintList candidates = zone->save ? zone->save->getFactionBlueprints(fac) :
                                       globals.shipLoader ? globals.shipLoader->getOrigBlueprints(fac) : BlueprintList();
            const bool isCurrentValid = mycl->isBlueprintValid();
            const float my_capacity = mycl->getResourceCapacity();

            for (int i=0; i<candidates.size(); )
            {
                const BlockCluster *bp = candidates[i];
                const bool valid =
                    (isCurrentValid && bp->getDeadliness() <= mycl->getDeadliness()) ||
                    (bp->getDeadliness() > my_capacity + sc->energy) ||
                    !mycl->isBlueprintValid(bp) ||
                    // FIXME need logic to determine if this is a good place to become a space station
                    (m_ai->isMobile() != bp->isMobile());
                vec_pop_increment(candidates, i, valid);
            }
            if (candidates.size()) {
                sc->nextprint.assign(selectMinRandomized(candidates, candidates.size(), BlockCluster::lessByDeadliness));
            }
        }
        if (!sc->nextprint)
            return noAction("can't find blueprint");
    }

    which = sc->nextprint->getName().c_str();

    if (command->cluster->size() > 1)
    {
        const float needRes = max(0, sc->nextprint->getDeadliness() - sc->energy);

        if (sc->resources < needRes)
            return noAction("not enough res");

        sc->resources -= needRes;
    }

    // do the brain transplant, and reset the ai
    // AHeal will take care of actually reassembling, after the reset
    sc->blueprint = std::move(sc->nextprint);
    sc->flags &= PERSISTENT_COMMAND_FLAGS;
    sc->finalize();
    command->sb.lifetime = -1;
    status = "metamorphasized";
    return LANE_ASSEMBLE;
}

vector<ResourcePocket*>& AI::getVisibleResources()
{
    if (m_visibleResources.empty())
    {
        const BlockCluster *cl = command->cluster;
        cl->zone->intersectCircleResources(m_visibleResources, cl->getAbsolutePos(), cl->getSensorRadius());
    }
    return m_visibleResources;
}

bool AI::isValidTarget(const Block *bl) const
{
    return exist(bl) &&
        (bl->sb.features&(Block::COMMAND|Block::EXPLODE)) &&
        bl->getFaction() != getFaction() &&
        bl->sb.command &&
        !bl->isVegetable();
}

float2 AI::estimateTargetPos() const
{
    return targetPos + float(zone->simTime - targetPosTime) * targetVel;
}

bool AI::canEstimateTargetPos() const
{
    return 0 < targetPosTime && (zone->simTime - targetPosTime) < kTargetTimeout;
}


const BlockList& AI::getEnemies()
{
    if (!m_enemiesQueried) {
        m_enemies.clear();
        const float radius = isMobile() ? 0.f : getAttackCaps().maxRange;
        command->cluster->zone->intersectCircleBlocks(
            &m_enemies, patternForTarget(command, Block::COMMAND, radius));
        vec_pop_unless(m_enemies, std::bind(&AI::isValidTarget, this, std::placeholders::_1));
        m_enemiesQueried = true;
    }
    return m_enemies;
}

const BlockList& AI::getAllies()
{
    if (!m_alliesQueried) {
        m_allies.clear();
        command->cluster->zone->intersectCircleBlocks(&m_allies, patternForAllies(command->cluster));
        m_alliesQueried = true;
    }
    return m_allies;
}

float AI::getEnemyDeadliness()
{
    m_renderEnemyDeadliness = 0;
    foreach (const Block* bl, getEnemies()) {
        m_renderEnemyDeadliness += bl->cluster->getDeadliness();
    }
    return m_renderEnemyDeadliness;
}

float AI::getAllyDeadliness()
{
    m_renderAllyDeadliness = command->cluster->getDeadliness();
    foreach (const Block* bl, getAllies()) {
        m_renderAllyDeadliness += bl->cluster->getDeadliness();
    }
    return m_renderAllyDeadliness;
}

float AI::getEnemyAllyRatio()
{
    float enemy = getEnemyDeadliness();
    if (enemy == 0.f)
        return 0.f;
    return enemy / getAllyDeadliness();
}

struct ASignalBackup final : public AIAction {

    ASignalBackup(AI* ai, float th, float th2) : AIAction(ai, LANE_SIGNAL),
                                                      threshold(th), signalThreshhold(th2) {}
    float ratio = 0.f;
    float threshold;
    float signalThreshhold;

    virtual uint update(uint blockedLanes)
    {
        ratio = m_ai->getEnemyAllyRatio();
        if (ratio < threshold) {
            return LANE_NONE;
        }

        m_ai->signal.reset(AISignal::DEADLINESS, m_ai->getEnemyDeadliness() / signalThreshhold,
                           1, getClusterPos(), 0.4f * getCluster()->getSensorRadius());
        return LANE_SIGNAL;
    }

    virtual string toStringEx() const
    {
        return str_format("%.2f >= %.2f ?%s", ratio, threshold,
                          (ratio >= threshold) ? " signaled" : "");
    }
};


// run away from enemies, if the ratio is above threshold
// FIXME it might be more effective and also more efficient to do this differently:
// FIXME sample to find a point on the enemy circle with no obstacles
// FIXME possibly also with velocity obstacles for scoring by away-from-enemies-ness
struct ARunAway final : public AIAction {

    ARunAway(AI* ai, float th) : AIAction(ai, LANE_MOVEMENT), path(ai), threshold(th) {}
    virtual void render(void* line) const { path.render(line); }

    APath path;
    float      ratio = 0.f;
    float      threshold;

    static bool supportsConfig(const AICommandConfig& cfg) { return APath::supportsConfig(cfg); }

    virtual uint update(uint blockedLanes)
    {
        const float2 myPos = getClusterPos();

        if (path.IsFinished || m_ai->isBigUpdate())
        {
            ratio = m_ai->getEnemyAllyRatio();

            if (path.IsFinished && ratio < threshold) {
                status = "not too dangerous";
                return LANE_NONE;
            }

            if (m_ai->getEnemies().size())
            {
                float2 enemyCenter;
                float  enemyCenterRad = 0;
                foreach (const Block* bl, m_ai->getEnemies()) {
                    const float2 pos = bl->getClusterPos();
                    enemyCenter    += pos;
                    enemyCenterRad  = max(enemyCenterRad,
                                          (length(pos - myPos) + bl->cluster->getSensorRadius()));
                }
                enemyCenter /= float(m_ai->getEnemies().size());
                enemyCenterRad += 4.f * getClusterBRadius();

                const float2 away = nearZero(myPos - enemyCenter) ? randpolar(1.f) : normalize(myPos - enemyCenter);
                float2 outsidePoint = enemyCenter + enemyCenterRad * away;
                path.setPathDest(outsidePoint, 4.f * getClusterBRadius());
                status = "fleeing enemies";
            }
            else
            {
                status = "still running";
            }
        }
        return path.update(0);
    }

    virtual string toStringEx() const
    {
        return str_format("%.1f:%.1f: %s", ratio, threshold, status);
    }
};


// set fallback target - i.e. incoming missile
uint AFallbackTarget::update(uint blockedLanes)
{
    if (m_ai->isBigUpdate())
    {
        float         minDistSqr = FLT_MAX;
        const Block  *target     = NULL;
        const float2  pos        = m_ai->command->getClusterPos();

        foreach (const Block* tgt, m_ai->getEnemies())
        {
            const float dist = distanceSqr(pos, tgt->getAbsolutePos());
            if (dist < minDistSqr && (isActivelyHostile(tgt) != AI::NEUTRAL)) {
                minDistSqr = dist;
                target     = tgt;
            }
        }
        foreach (const Block *tgt, m_ai->getQuery().queryBlockObstacles(m_ai->command))
        {
            const float dist = distanceSqr(pos, tgt->getAbsolutePos());
            if (dist < minDistSqr && (isActivelyHostile(tgt) != AI::NEUTRAL)) {
                minDistSqr = dist;
                target     = tgt;
            }
        }

        status = target ? "Set fallback target" : "no fallback target";
        m_ai->fallbackTarget = target;
    }
    return LANE_NONE;       // never blocks other targetingp
}


AIMood ATargetBase::acceptTarget(const Block* target) const { return AIMood::OFFENSIVE; }

ATargetBase::Target ATargetBase::testAcceptTarget(const Block *tgt) const
{
    if (!tgt || !live(tgt) || !tgt->isCommand())
        return Target();
    const AIMood mood = acceptTarget(tgt);
    if (mood == AIMood::NEUTRAL)
        return Target();
    return make_pair(tgt, mood);
}

float ATargetBase::targetDistanceMetric(float2 defPos, const Block *tgt) const
{
    const AttackCapabilities &caps = m_ai->getAttackCaps();
    const float2 tpos = tgt->getAbsolutePos();
    float dist = distanceSqr(defPos, tpos);
    // if we have fixed weapons, weight enemies in front more heavily
    if (caps.hasFixed)
        dist *= 1.f / (2.f + dot(getCluster()->getRot(), normalize(tpos - defPos)));
    return dist;
}

uint ATargetBase::findSetTarget()
{
    if (!m_ai->getAttackCaps().weapons)
    {
        status = "No Weapons";
        m_ai->setTarget(NULL, AIMood::NEUTRAL);
        return LANE_TARGET;
    }

    Target target = testAcceptTarget(m_ai->priorityTarget.get());
    if (!target.first)
        target = testAcceptTarget(m_ai->target.get());

    if (!(target.first && m_ai->priorityTarget) && m_ai->isBigUpdate())
    {
        targets.clear();
        const bool isAttack = (m_ai->getConfig().flags&ECommandFlags::ATTACK);
        const int deadlyThreshold = isAttack ? 10 : min(kAITargetMin, int(kAITargetThreshold * getCluster()->getDeadliness()));
        foreach (const Block* tgt, m_ai->getEnemies())
        {
            AIMood mood = AIMood::NEUTRAL;
            if (tgt->getBlueprintDeadliness() >= deadlyThreshold &&
                (!isAttack || !tgt->sb.isTransient()) &&
                (mood = acceptTarget(tgt)) != AI::NEUTRAL)
            {
                targets.push_back(make_pair(tgt, mood));
            }
        }

        // pick closest remaining target
        const float2 defPos = !nearZero(m_ai->defendPos) ? m_ai->defendPos : getClusterPos();
        target = vec_min(
            targets, [&](const Target& tgt) { return targetDistanceMetric(defPos, tgt.first); },
            target, (target.first ? (0.7 * targetDistanceMetric(defPos, target.first)) :
                     FLT_MAX));
    }

    if (!target.first)
        return noAction("No Target");

    status = "Found Target";
    m_ai->setTarget(target.first, target.second);
    return LANE_TARGET;
}


// target nearest offensive enemy
struct ATargetDefense final : public ATargetBase {

    ATargetDefense(AI* ai) : ATargetBase(ai) {}

    virtual AI::AIMood acceptTarget(const Block* target) const
    {
        return isActivelyHostile(target);
    }

    virtual uint update(uint blockedLanes)
    {
        return findSetTarget();
    }
};


// target nearest allies (be "offensive" toward them)
struct ATargetHelpAllies final : public ATargetBase {

    ATargetHelpAllies(AI* ai) : ATargetBase(ai) {}

    virtual AI::AIMood acceptTarget(const Block* target) const
    {
        return (target->commandAI &&
                target->commandAI->mood != AI::NEUTRAL &&
                target->commandAI->moodFaction == m_ai->command->getFaction()) ?
            AI::OFFENSIVE : AI::NEUTRAL;
    }

    virtual uint update(uint blockedLanes)
    {
        return findSetTarget();
    }
};

struct ATargetHereditary final : public ATargetBase {

    ATargetHereditary(AI* ai) : ATargetBase(ai) {}

    virtual uint update(uint blockedLanes)
    {
        if (m_ai->priorityTarget)
            return noAction("Has priority target");
        const AI* pai = m_ai->getParentAI();
        if (!pai)
            return noAction("No Parent");
        const Block *tgt = or_(pai->priorityTarget.get(), pai->target.get()); // not fallback target!
        const Block *ftgt = pai->fallbackTarget.get();
        if (!tgt && !ftgt)
            return noAction("Parent has no target");

#if 0
        const BlockCluster *cl = getCluster();
        if (!intersectCircleCircle(tgt->getClusterPos(), tgt->getClusterBRadius(),
                                   cl->getAbsolutePos(), 2.f * cl->getSensorRadius()))
        {
            return noAction("Too far");
        }
#endif

        if (tgt && tgt->isCommand()) {
            m_ai->setTarget(tgt, pai->mood);
        } else {
            m_ai->fallbackTarget = or_(tgt, ftgt);
        }
        return LANE_TARGET;
    }
};

struct AUntarget final : public AIAction {

    AUntarget(AI* ai) : AIAction(ai, LANE_TARGET, PRI_DEFAULT) {}

    virtual uint update(uint blockedLanes)
    {
        m_ai->setTarget(NULL, AI::NEUTRAL);
        return LANE_TARGET;
    }
};

struct ATargetOpportunity final : public AIAction {

    ATargetEnemy targetEnemy;
    const float       threshold;

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return cfg.usesResources();
    }

    ATargetOpportunity(AI* ai, float th) : AIAction(ai, LANE_TARGET, PRI_DEFAULT),
                                              targetEnemy(ai), threshold(th)  {}

    virtual uint update(uint blockedLanes)
    {
        float ratio = m_ai->getEnemyAllyRatio();
        if (ratio > threshold)
            return noAction("Too dangerous");

        return targetEnemy.update(0);
    }

};

bool AWeapons::supportsConfig(const AICommandConfig& cfg)
{
    return cfg.hasWeapons && (cfg.features&FIREABLE_WEAPONS);
}

uint AWeapons::update(uint blockedLanes)
{
    enabled = 0;
    if (m_ai->target) {
        FiringData data(m_ai->target);
        data.filter = ~Block::AUTOFIRE;
        enabled = m_ai->fireWeaponsAt(data);
        isFallback = false;
    }
    if (!enabled && m_ai->fallbackTarget && nearZero(m_ai->rushDir)) {
        FiringData data(m_ai->fallbackTarget);
        data.filter = ~Block::AUTOFIRE;
        enabled = m_ai->fireWeaponsAt(data);
        isFallback = true;
    }
    return enabled ? LANE_SHOOT : LANE_NONE;
}

bool APositionBase::isTargetObstructed(const Block *target)
{
    if (!target || !target->commandAI) {
        status = "No Target";
        return true;
    }

    if ((m_ai->isBigUpdate() &&
         (obstructed = isDestObstructed(target->getClusterPos(), target->getFaction(), 1.f))) ||
        obstructed)
    {
        status = "Obstructed";
        return true;
    }

    return false;
}


struct ARamTarget final : public APositionBase {

    ARamTarget(AI* ai): APositionBase(ai) {}

    static bool supportsConfig(const AICommandConfig& cfg)
    {
        return (cfg.features&(Block::EXPLODE|Block::MELEE)) && AMove::supportsConfig(cfg);
    }

    virtual uint update(uint blockedLanes)
    {
        const Block *target = m_ai->getTarget();
        if (isTargetObstructed(target))
            return LANE_NONE;

        snConfig cfg;
        target->getNavConfig(&cfg);
        m_ai->nav->setDest(cfg, SN_POSITION|SN_TARGET_VEL, 0);
        return LANE_MOVEMENT;
    }
};

bool AAttack::supportsConfig(const AICommandConfig& cfg)
{
    return cfg.hasWeapons && (cfg.features&FIREABLE_WEAPONS) && AMove::supportsConfig(cfg);
}

uint AAttack::update(uint blockedLanes)
{
    m_ai->rushDir = float2();
    const Block *target = m_ai->target.get();
    if (isTargetObstructed(target))
        return LANE_NONE;

    target->getNavConfig(&targetCfg.cfg);
    snPrecision precision;
    precision.pos = getWaypointRadius();

    Block*        command = m_ai->command;
    BlockCluster* cluster = command->cluster;

    const AttackCapabilities &caps = m_ai->getAttackCaps();

    // can't attack without weapons...
    if (!caps.weapons)
        return noAction("No Weapons");

    const AttackCapabilities &tcaps = target->commandAI->getAttackCaps();

    const float2 pos        = cluster->getAbsolutePos();
    const float2 targetPos  = target->getAbsolutePos();
    const float2 targetVel  = target->cluster->getVel();
    const float  targetDist = distance(targetPos, pos) - 0.5f * target->cluster->getCoreRadius();

    // FIXME targetDist is a hack here... works fine for the common case

    targetCfg.dims = 0;

    const float mydps = caps.rushDps / tcaps.totalHealth;
    const float tdps = tcaps.totalDps / caps.totalHealth;
    const uint64 flags = m_ai->getConfig().flags;

    const bool rushing = (mydps > 1.1f * tdps || (flags&SerialCommand::ALWAYS_RUSH)) &&
                         !(flags&(SerialCommand::ALWAYS_MANEUVER|SerialCommand::ALWAYS_KITE));
    const float snipeRange = 1.1f * tcaps.maxRange;
    const bool canStayOutOfRange = (caps.maxRange > snipeRange) &&
                                   target->cluster->isMobile() &&
                                   caps.getDpsAtRange(snipeRange) > 2.f * tcaps.healthRegen &&
                                   !(flags&SerialCommand::ALWAYS_MANEUVER);
    const bool sniping = (!rushing && canStayOutOfRange) || (flags&SerialCommand::ALWAYS_KITE);
    status = rushing ? _("Rushing") :
             canStayOutOfRange ? gettext_("Kiting", "Sniping") : _("Maneuvering");

    // FIXME too many magic numbers here! at least make them cvars
    const float wantRange = rushing ? 0.f :
        canStayOutOfRange ? snipeRange :
        (0.9f * caps.bestRange);

    const float2 targetLeadPos = targetPos + kAIBigTimeStep * targetVel;
    const float2 targetDir = normalize(targetLeadPos - pos);

    if (!canStayOutOfRange && wantRange < targetDist)
        m_ai->rushDir = targetDir;

    const float2 dir = (caps.hasFixed ? directionForFixed(cluster, targetPos, targetVel, FiringFilter()) :
                        targetLeadPos - pos);

    // move to the optimal attack range
    targetCfg.cfg.position = targetLeadPos - targetDir * wantRange;
    targetCfg.cfg.velocity = 0.95f * targetVel; // damp velocity - otherwise they drift together
    targetCfg.cfg.angle = vectorToAngle(dir);
    targetCfg.dims = SN_POSITION | SN_ANGLE | (rushing ? SN_TARGET_VEL : SN_VELOCITY);
    precision.pos = max(precision.pos, 0.1f * caps.bestRange);

    // escape super fast if we are sniping
    if (sniping) {
        if (targetDist < wantRange) {
            targetCfg.cfg.velocity += 10.f * (targetCfg.cfg.position - pos);
            targetCfg.dims = SN_ANGLE | SN_VELOCITY | SN_VEL_ALLOW_ROTATION;
        }
        else if (targetDist < 1.1f * caps.maxRange) {
            // don't worry about position, just match velocity
            if (caps.hasFixed)
                targetCfg.dims = SN_ANGLE | SN_VELOCITY;
            else
                targetCfg.dims = SN_ANGLE | SN_VELOCITY | SN_VEL_ALLOW_ROTATION;
        }
    }
    else if (caps.hasFixed && targetDist <= caps.maxRange) {
        targetCfg.dims |= SN_POS_ANGLE;
    }

    if (!targetCfg.dims)
        return noAction("No direction");

    m_ai->nav->setDest(targetCfg.cfg, targetCfg.dims, precision);
    return LANE_MOVEMENT;
}

uint AHealers::update(uint blockedLanes)
{
    if (m_ai->isBigUpdate())
    {
        float range = 0.f;
        bool fixed = false;
        foreach (const Block* bl, getCluster()->blocks)
        {
            if (!bl->sb.isHealer())
                continue;
            range = max(range, bl->sb.weaponRange());
            if (bl->sb.isFireableFixed())
                fixed = true;
        }

        const float2 mypos = getClusterPos();
        float mindist = FLT_MAX;
        const Block *target = NULL;
            
        foreach (const Block* tgt, m_ai->getAllies())
        {
            if (tgt == m_ai->command || tgt->sb.isTransient() || (tgt->sb.features&Block::PERISHABLE))
                continue;
            const BlockCluster *tcl = tgt->cluster;
            if (tcl->isFullHealth())
                continue;
            if (test_min(distance(tcl->getAbsolutePos(), mypos) - tcl->getBRadius(), &mindist))
                target = tgt;
        }

        m_ai->healTarget = target;
        m_ai->healRange = range;
        fixedHealer = fixed;
    }

    if (!m_ai->healTarget)
        return LANE_NONE;
    FiringData data(m_ai->healTarget);
    data.filter.healer = true;
    uint lanes = LANE_NONE;
    foreach (Block* bl, getCluster()->blocks)
    {
        if (!bl->sb.isHealer())
            continue;
        if (!bl->fireWeapon(data))
            continue;
        // also block movement lane to prevent moving away while healing
        lanes |= LANE_HEALER|LANE_MOVEMENT;
    }
    
    if (!intersectCircleCircle(getClusterPos(), m_ai->healRange, data.clusterPos, data.clusterRad) ||
        !m_ai->getConfig().isMobile ||
        (blockedLanes&LANE_MOVEMENT))
        return lanes;

    snConfigDims cfg;
    cfg.dims = SN_VELOCITY;
    cfg.cfg.velocity = data.clusterVel;
    if (fixedHealer) {
        cfg.cfg.angle = v2a(directionForFixed(getCluster(), data));
        cfg.dims |= SN_ANGLE;
    }
    m_ai->nav->setDest(cfg.cfg, cfg.dims, snPrecision());
    return lanes|LANE_MOVEMENT;
}

string AIActionList::toString() const
{
    string s;
    for (uint i=0; i<actions.size(); i++)
    {
        s += (actionWasRun&(1<<i)) ? " * " : "   ";
        s += actions[i]->toString();
        if (i != actions.size() - 1)
            s += "\n";
    }
    if (actions.size() == 0)
    {
        s += "<no ai actions>";
    }
    return s;
}

size_t AIActionList::getSizeof() const
{
    size_t sz = sizeof(*this);
    for_ (ac, actions)
        sz += sizeof(*ac);
    return sz;
}

void AIActionList::render(VertexPusherLine &vp) const
{
    for (uint i=0; i<actions.size(); i++)
    {
        if (actionWasRun&(1<<i) && actions[i]->Blocking)
            actions[i]->render((void*)&vp);
    }
}

void AIActionList::insert(AIAction* a)
{
    for (uint i=0; i<actions.size(); i++)
    {
        if (a->Priority < actions[i]->Priority)
        {
            actions.insert(actions.begin() + i, a);
            return;
        }
    }
    actions.push_back(a);
}

int AIActionList::clearTag(uint tag)
{
    int count = 0;
    for (uint i=0; i<actions.size(); )
    {
        if (actions[i]->Tag&tag)
        {
            count++;
            delete actions[i];
            actions.erase(actions.begin() + i);
        }
        else
            i++;
    }
    return count;
}

void AIActionList::clear()
{
    ASSERT(lastActions.empty());
    vec_clear_deep(actions);
}


void AIActionList::update()
{
    actionWasRun = 0;
    prettyState = "";

    int mask = 0;
    for (uint i=0; i != actions.size(); )
    {
        AIAction *action = actions[i];
        if ((mask & action->Lanes) == 0)
        {
            action->Blocking  = action->update(mask);
            mask             |= action->Blocking;
            actionWasRun     |= 1<<i;

            if (action->Blocking&AIAction::LANE_MOVEMENT)
                prettyState = action->toPrettyString();

            if (action->IsFinished)
            {
                delete action;
                actions.erase(actions.begin() + i);
                continue;
            }
        }
        else
        {
            action->Blocking = 0;
        }
        i++;
    }
}

AICommandConfig::AICommandConfig(const Block* cmd)
{
    flags      = cmd->sb.command->flags.get();
    features   = cmd->cluster->getFeatureUnion();
    isMobile   = cmd->cluster->isMobile();
    isDoomed   = cmd->sb.lifetime != -1.f;
    isAttached = cmd->cluster->parent;
    hasFreeRes = cmd->hasFreeRes();
    hasParent  = cmd->getAIParent();
    hasWeapons = vec_any(cmd->cluster->blocks, [](const Block *bl) { return bl->sb.isWeapon(); });
    hasHealers = vec_any(cmd->cluster->blocks, [](const Block *bl) { return bl->sb.isHealer(); });
}

bool AICommandConfig::usesResources() const
{
    return (features&Block::TRACTOR) && !hasFreeRes;
}

bool AICommandConfig::isRoot() const
{
    return (features&(Block::SEED|Block::ROOT));
}

bool AICommandConfig::operator==(const AICommandConfig& o) const
{
    return flags   == o.flags      &&
        features   == o.features   &&
        isMobile   == o.isMobile   &&
        isDoomed   == o.isDoomed   &&
        isAttached == o.isAttached &&
        hasFreeRes == o.hasFreeRes &&
        hasParent  == o.hasParent &&
        hasWeapons == o.hasWeapons &&
        hasHealers == o.hasHealers;
}

AIAction *AI::getRecycle(const std::type_info &tp)
{
    for (int i=0; i< m_actions.lastActions.size(); )
    {
        AIAction *ac = m_actions.lastActions[i];
        if (vec_pop_increment(m_actions.lastActions, i, typeid(*ac) == tp)) {
            ac->onReset();
            return ac;
        }
    }
    return NULL;
}

template <typename T>
void AI::addRecycleAction()
{
    if (!T::supportsConfig(m_config))
        return;
    AIAction *ac = getRecycle(typeid(T));
    m_actions.insert(ac ? ac : new T(this));
}


#define ADD_ACTION(TYPE) addRecycleAction<TYPE>()

#if CHRIS_DLL_TEST
#define ADD_ACTION_MODDABLE(TYPE)                                   \
    if (TYPE::supportsConfig(m_config))                             \
    {                                                               \
        bool addedAction = false;                                   \
        if (kAiMod && m_aiModCreateAction)                          \
        {                                                           \
            if (AIAction * ac = m_aiModCreateAction(#TYPE, this))   \
            {                                                       \
                m_actions.insert(ac);                               \
                addedAction = true;                                 \
            }                                                       \
        }                                                           \
                                                                    \
        if (!addedAction)                                           \
        {                                                           \
            ADD_ACTION(TYPE);                                       \
        }                                                           \
    }
#else
#define ADD_ACTION_MODDABLE(TYPE) ADD_ACTION(TYPE)
#endif

#define ADD_ACTION_(TYPE, ...)                          \
    if (TYPE::supportsConfig(m_config))                 \
        m_actions.insert(new TYPE(this, __VA_ARGS__))


// personality flags
ECommandFlags SerialCommand::getInitialFlags() const
{
    const FactionData *fac = globals.shipLoader->getFactionData(faction);
    ECommandFlags flgs = fac ? fac->aiflags : (FORGIVING|WANDER|DODGES);

    if (!kAIRandomizeFlags)
        return flgs;

    if (randrange(0, 2) == 0)
        flgs ^= RIPPLE_FIRE;
    if (randrange(0, 2) == 0)
        flgs ^= SMART_FIRE;
    if (randrange(0, 4) == 0)
        flgs ^= BAD_AIM;

    if (randrange(0, 5) == 0)
        flgs ^= RECKLESS;
    else if (randrange(0, 5) == 0)
        flgs ^= CAUTIOUS;
    else if (randrange(0, 5) == 0)
        flgs ^= NO_PARENT;

    if (randrange(0, 10) == 0)
        flgs ^= FLOCKING;
    if (randrange(0, 10) == 0)
        flgs ^= DODGES;
    // if (randrange(0, 10) == 0)
        // flgs ^= WANDER;
    if (randrange(0, 10) == 0)
        flgs ^= PEACEFUL;

    if (flgs&METAMORPHOSIS) {
        if (randrange(0, 4) != 0)
            flgs ^= METAMORPHOSIS;
    } else if (randrange(0, 10) == 0)
        flgs ^= METAMORPHOSIS;

    return flgs;
}

#if CHRIS_DLL_TEST
bool AI::recreateActionsModded()
{
    if (!m_aiModCreateActions)
        return false; // no AI mod setup for this AI

    m_actions.actions.size();
    swap(m_actions.actions, m_actions.lastActions);
    mood = NEUTRAL;
    moodFaction = 0;
    m_attackCaps.initialized = false;
    ASSERT(live(command));
    ASSERT(sc == command->sb.command.get());

    if (!sc->faction)
        sc->faction = command->sb.group;
    if (!(sc->flags&~PERSISTENT_COMMAND_FLAGS)) {
        sc->flags = sc->getInitialFlags()|(sc->flags&PERSISTENT_COMMAND_FLAGS);
        const BlockCluster *bp = command->getBlueprint();
        if (bp && bp->command)
            sc->flags |= bp->command->sb.command->flags;
        if (sc->flags&SerialCommand::FOLLOWER)
            sc->flags &= ~FOLLOWER_NOFLAGS;
        m_config.flags = sc->flags.get();
    }
    if (!sc->ident)
        sc->ident = randrange<uint>();

    const ECommandFlags::value_type flags = m_config.flags;

    if ((flags&SerialCommand::NONE) || (m_config.features&Block::UNGROW)) {
        vec_clear_deep(m_actions.lastActions);
        return true;
    }

    /*
    // missiles, spikes
    if (((m_config.features&Block::EXPLODE) ||
         ((m_config.features&WEAPON_FEATURES) == Block::MELEE)) &&
        !(m_config.features&(Block::ASSEMBLER|Block::REGROWER)))
    {
        if (flags&SerialCommand::POINT_DEFENSE) {
            ADD_ACTION(AFallbackTarget);
            ADD_ACTION(ATargetDefense);
        } else {
            ADD_ACTION(ATargetHereditary);
        }
        ADD_ACTION(ATargetEnemy);
        ADD_ACTION(AWeapons);
        ADD_ACTION(ARamTarget);
        ADD_ACTION(AInvestigate);
        if (flags&SerialCommand::POINT_DEFENSE) {
            ADD_ACTION(AFollowParent);
        } else if (m_config.isDoomed) {
            ADD_ACTION(AJustGo);
        }
        vec_clear_deep(m_actions.lastActions);
        return true;
    }

    const BlockCluster *blueprint = command->cluster->getCreateBlueprint();
    // const BlockCluster *blueprint = command->cluster->getBlueprint();

    const float fleeRatio   = (flags&SerialCommand::RECKLESS) ? 3.f : 1.5;
    const float attackRatio = m_config.hasHealers ? 0.3f :
                              (flags&SerialCommand::AGGRESSIVE) ? 1.2f :
                              (flags&SerialCommand::CAUTIOUS) ? 0.3f : 0.75f;

    const uint64 zoneFeatures = command->cluster->zone->getZoneFeatures().get();
    */

    /*
    if (m_config.isMobile >= 2 && (m_config.flags&SerialCommand::DODGES))
    {
        ADD_ACTION_MODDABLE(AAvoidWeapon);
    }

    ADD_ACTION_MODDABLE(AWeapons);

    /////
    // tournament mode
    ADD_ACTION_MODDABLE(AFallbackTarget);
    ADD_ACTION_MODDABLE(ATargetEnemy);
    ADD_ACTION_MODDABLE(AAvoidCluster);
    ADD_ACTION_MODDABLE(AAttack);
    ADD_ACTION_MODDABLE(AHealers);
    ADD_ACTION_MODDABLE(AInvestigate);
    /////
    */

    m_aiModCreateActions(this);

    /*
    if (m_config.features&Block::ASSEMBLER)
    {
        ADD_ACTION(AHeal);
        if (m_config.flags&SerialCommand::TRACTOR_TRANSIENT) {
            ADD_ACTION(AScavengeWeapon);
        }
        if (!m_config.hasFreeRes || kAIEnableNoResReproduce)
        {
            if (m_config.flags&SerialCommand::METAMORPHOSIS) {
                ADD_ACTION(AMetamorphosis);
            }
            ADD_ACTION(ABudReproduce);
        }
        // else ADonate: find allies and give them resources?
    }
    else if (m_config.features&Block::REGROWER)
    {
        ADD_ACTION(AHeal);
    }

    if (m_config.isMobile && m_config.isRoot() && !m_config.isAttached)
    {
        // FIXME need to either metamorphasize or plant self
        // FIXME very bad to overwrite to non-plant blueprint and then get planted
        ADD_ACTION(APlantSelf);
        ADD_ACTION(AMetamorphosis);
    }

    if (m_config.isMobile && !nearZero(command->sb.command->destination))
    {
        appendCommandDest(command->sb.command->destination, 0.25f * kSectorSize);
    }

    if (m_config.isMobile &&
        !(m_config.flags&(SerialCommand::FOLLOWER)) &&
        !m_config.hasParent &&
        (m_config.flags&SerialCommand::WANDER))
    {
        ADD_ACTION(AWander);
    }
    */

    /*
    DPRINT(AI, ("Reused %d/%d", startCount - (int)m_actions.lastActions.size(),
                (int)m_actions.actions.size()));
    */
    vec_clear_deep(m_actions.lastActions);
    return true;
}
#else
bool AI::recreateActionsModded() { return false; }
#endif

void AI::addAction(AIAction * action)
{
    m_actions.insert(action);
}

void AI::recreateActions()
{
    {
        const AICommandConfig cfg = AICommandConfig(command);
        if (m_config == cfg && m_actions.actions.size())
            return;
        m_config = cfg;
    }

    if (kAiMod && recreateActionsModded())
        return;

    const int startCount = m_actions.actions.size();
    swap(m_actions.actions, m_actions.lastActions);
    mood = NEUTRAL;
    moodFaction = 0;
    m_attackCaps.initialized = false;
    ASSERT(live(command));
    ASSERT(sc == command->sb.command.get());

    if (!sc->faction)
        sc->faction = command->sb.group;
    if (!(sc->flags&~PERSISTENT_COMMAND_FLAGS)) {
        sc->flags = sc->getInitialFlags()|(sc->flags&PERSISTENT_COMMAND_FLAGS);
        const BlockCluster *bp = command->getBlueprint();
        if (bp && bp->command)
            sc->flags |= bp->command->sb.command->flags;
        if (sc->flags&SerialCommand::FOLLOWER)
            sc->flags &= ~FOLLOWER_NOFLAGS;
        m_config.flags = sc->flags.get();
    }
    if (!sc->ident)
        sc->ident = randrange<uint>();

    const ECommandFlags::value_type flags = m_config.flags;

    if ((flags&SerialCommand::NONE) || (m_config.features&Block::UNGROW)) {
        vec_clear_deep(m_actions.lastActions);
        return;
    }

    // missiles, spikes
    if (((m_config.features&Block::EXPLODE) ||
         ((m_config.features&WEAPON_FEATURES) == Block::MELEE)) &&
        !(m_config.features&(Block::ASSEMBLER|Block::REGROWER)))
    {
        if (flags&SerialCommand::POINT_DEFENSE) {
            ADD_ACTION(AFallbackTarget);
            ADD_ACTION(ATargetDefense);
        } else {
            ADD_ACTION(ATargetHereditary);
        }
        ADD_ACTION(ATargetEnemy);
        ADD_ACTION(AWeapons);
        ADD_ACTION(ARamTarget);
        ADD_ACTION(AInvestigate);
        //ADD_ACTION(AWander);
        //ADD_ACTION(AStop);
        if (flags&SerialCommand::POINT_DEFENSE) {
            ADD_ACTION(AFollowParent);
        } else if (m_config.isDoomed) {
            ADD_ACTION(AJustGo);
        }
        vec_clear_deep(m_actions.lastActions);
        return;
    }

    const BlockCluster *blueprint = command->cluster->getCreateBlueprint();
    // const BlockCluster *blueprint = command->cluster->getBlueprint();

    const float fleeRatio   = (flags&SerialCommand::RECKLESS) ? 3.f : 1.5;
    const float attackRatio = m_config.hasHealers ? 0.3f :
                              (flags&SerialCommand::AGGRESSIVE) ? 1.2f :
                              (flags&SerialCommand::CAUTIOUS) ? 0.3f : 0.75f;

    const uint64 zoneFeatures = command->cluster->zone->getZoneFeatures().get();


    if (m_config.isMobile >= 2 && (flags&SerialCommand::DODGES))
    {
        ADD_ACTION(AAvoidWeapon);
    }

    ADD_ACTION(AWeapons);

    if (m_config.isDoomed && !m_config.isRoot())
    {
        // drones, deployables
        if ((flags&SerialCommand::PACIFIST)) {
            ADD_ACTION(AUntarget);
        } else {
            ADD_ACTION(AFallbackTarget);
            if (flags&SerialCommand::FIRE_AT_WILL) {
                ADD_ACTION_(ATargetOpportunity, 4.f);
            } else if (!(flags&SerialCommand::POINT_DEFENSE)) {
                ADD_ACTION(ATargetHereditary);
            }
            ADD_ACTION(ATargetDefense);
            ADD_ACTION(ATargetHelpAllies);
        }
        ADD_ACTION(AAttack);
        ADD_ACTION(AHealers);
        ADD_ACTION(AInvestigateHereditary);
        ADD_ACTION(AInvestigate);
        ADD_ACTION(AResToParent);
        ADD_ACTION(AFollowParent);
    }
    else if (flags&SerialCommand::ATTACK)
    {
        // tournament mode
        ADD_ACTION(AFallbackTarget);
        ADD_ACTION(ATargetEnemy);
        ADD_ACTION(AAvoidCluster);
        ADD_ACTION(AAttack);
        ADD_ACTION(AHealers);
        ADD_ACTION(AInvestigate);
    }
    else if (flags&SerialCommand::HANGOUT)
    {
        ADD_ACTION(ARotate);
    }
    else
    {
        const bool follower = (flags&SerialCommand::FOLLOWER);
        const bool target_follower = follower && !(flags&SerialCommand::FIRE_AT_WILL);

        ADD_ACTION(AAvoidCluster);
        if (flags&SerialCommand::SOCIAL) {
            ADD_ACTION_(ASignalBackup, 0.75f, 0.5f);
        }
        if (flags&SerialCommand::FOLLOWER) {
            ADD_ACTION(AReturnParent);
        }

        if (m_config.hasWeapons && !(flags&SerialCommand::PACIFIST)) {
            ADD_ACTION(AFallbackTarget);
            if (target_follower) {
                ADD_ACTION(ATargetHereditary);
            }
            if (m_config.isMobile) {
                ADD_ACTION(ATargetDefense);
            } else {
                ADD_ACTION(ATargetHelpAllies);
            }
            if (flags&SerialCommand::DODGES) {
                ADD_ACTION_(ARunAway, fleeRatio);
            }
            ADD_ACTION(AAttack);
        } else {
            if (flags&SerialCommand::DODGES) {
                ADD_ACTION_(ARunAway, 0.5f * fleeRatio);
            }
        }
        ADD_ACTION(AHealers);
        if (flags&SerialCommand::SOCIAL) {
            ADD_ACTION_(ARespondToSignal, AISignal::DEADLINESS);
        }
        if (m_config.hasWeapons | m_config.hasHealers) {
            if (target_follower) {
                ADD_ACTION(AInvestigateHereditary);
            } else {
                ADD_ACTION(AInvestigate);
                ADD_ACTION(AInvestigateHereditary);
            }
        }
        ADD_ACTION(ACollect);
        if (flags&SerialCommand::HATES_PLANTS) {
            ADD_ACTION(AKillPlants);
        }
        ADD_ACTION(AClearDebris);
        if (blueprint && APlant::getSeedLaunch(blueprint, NULL, NULL)) {
            ADD_ACTION(APlant);
        } else if (m_config.hasWeapons && !(flags&SerialCommand::PEACEFUL) && !target_follower)
        {
            if (zoneFeatures&GameZone::RESOURCES) {
                ADD_ACTION_(ATargetOpportunity, attackRatio);
            } else {
                ADD_ACTION(ATargetEnemy);
            }
        }
        if (m_config.hasWeapons && (flags&(SerialCommand::FORGIVING|SerialCommand::PACIFIST))) {
            ADD_ACTION(AUntarget);
        }
        if (flags&SerialCommand::SOCIAL) {
            ADD_ACTION_(ARespondToSignal, AISignal::CAPACITY);
        }

        if (flags&SerialCommand::FOLLOWER) {
            ADD_ACTION(AResToParent);
            ADD_ACTION(AFollowParent);
        } else {
            ADD_ACTION_(ACollectWander, 0.f);

            if (m_config.hasParent)
            {
                ADD_ACTION(AResToParent);
                ADD_ACTION(AFollowParent);
            }
            else if (flags&SerialCommand::FLOCKING)
            {
                ADD_ACTION(AFollowAllies);
            }
        }
    }

    if (m_config.features&Block::ASSEMBLER)
    {
        ADD_ACTION(AHeal);
        if (flags&SerialCommand::TRACTOR_TRANSIENT) {
            ADD_ACTION(AScavengeWeapon);
        }
        if (!m_config.hasFreeRes || kAIEnableNoResReproduce)
        {
            if (flags&SerialCommand::METAMORPHOSIS) {
                ADD_ACTION(AMetamorphosis);
            }
            ADD_ACTION(ABudReproduce);
        }
        // else ADonate: find allies and give them resources?
    }
    else if (m_config.features&Block::REGROWER)
    {
        ADD_ACTION(AHeal);
    }

    if (m_config.isMobile && m_config.isRoot() && !m_config.isAttached)
    {
        // FIXME need to either metamorphasize or plant self
        // FIXME very bad to overwrite to non-plant blueprint and then get planted
        ADD_ACTION(APlantSelf);
        ADD_ACTION(AMetamorphosis);
    }

    if (m_config.isMobile && !nearZero(command->sb.command->destination))
    {
        appendCommandDest(command->sb.command->destination, 0.25f * kSectorSize);
    }

    if (m_config.isMobile &&
        !(flags&(SerialCommand::FOLLOWER)) &&
        !m_config.hasParent &&
        (flags&SerialCommand::WANDER))
    {
        ADD_ACTION(AWander);
    }

    DPRINT(AI, ("Reused %d/%d", startCount - (int)m_actions.lastActions.size(),
                (int)m_actions.actions.size()));
    vec_clear_deep(m_actions.lastActions);
}

bool AI::addActionVanilla(VanillaActionType actionType)
{
#   define VANILLA_SUPPORT(SHORTNAME, TYPE, ...) \
        case VANILLA_ACTION_TYPE_##SHORTNAME: \
            if (TYPE::supportsConfig(m_config)) \
            { \
                addAction(new TYPE(this, ##__VA_ARGS__)); \
            } \
            return true

    switch (actionType)
    {
        VANILLA_SUPPORT(ATTACK,                 AAttack);
        VANILLA_SUPPORT(AVOID_CLUSTER,          AAvoidCluster);
        VANILLA_SUPPORT(AVOID_WEAPON,           AAvoidWeapon);
        VANILLA_SUPPORT(BUD_REPRODUCE,          ABudReproduce);
        //VANILLA_SUPPORT(CLEAR_DEBRIS,           AClearDebris);
        //VANILLA_SUPPORT(COLLECT,                ACollect);
        //VANILLA_SUPPORT(COLLECT_WANDER,         AWander);
        VANILLA_SUPPORT(FALLBACK_TARGET,        AFallbackTarget);
        //VANILLA_SUPPORT(FOLLOW_ALLIES,          AFollowAllies);
        //VANILLA_SUPPORT(FOLLOW_PARENT,          AFollowParent);
        VANILLA_SUPPORT(HEAL,                   AHeal);
        VANILLA_SUPPORT(HEALERS,                AHealers);
        VANILLA_SUPPORT(INVESTIGATE,            AInvestigate);
        //VANILLA_SUPPORT(INVESTIGATE_HEREDITARY, AInvestigateHereditary);
        //VANILLA_SUPPORT(JUST_GO,                AJustGo);
        //VANILLA_SUPPORT(KILL_PLANTS,            AKillPlants);
        VANILLA_SUPPORT(METAMORPHOSIS,          AMetamorphosis);
        //VANILLA_SUPPORT(PLANT,                  APlant);
        VANILLA_SUPPORT(PLANT_SELF,             APlantSelf);
        //VANILLA_SUPPORT(RAM_TARGET,             ARamTarget);
        //VANILLA_SUPPORT(RESPOND_TO_SIGNAL,      ARespondToSignal);
        //VANILLA_SUPPORT(RES_TO_PARENT,          AResToParent);
        //VANILLA_SUPPORT(RETURN_PARENT,          AReturnParent);
        //VANILLA_SUPPORT(ROTATE,                 ARotate);
        //VANILLA_SUPPORT(RUN_AWAY,               ARunAway);
        VANILLA_SUPPORT(SCAVENGE_WEAPON,        AScavengeWeapon);
        //VANILLA_SUPPORT(SIGNAL_BACKUP,          ASignalBackup);
        //VANILLA_SUPPORT(TARGET_DEFENSE,         ATargetDefense);
        VANILLA_SUPPORT(TARGET_ENEMY,           ATargetEnemy);
        //VANILLA_SUPPORT(TARGET_HELP_ALLIES,     ATargetHelpAllies);
        //VANILLA_SUPPORT(TARGET_HEREDITARY,      ATargetHereditary);
        //VANILLA_SUPPORT(TARGET_OPPORTUNITY,     ATargetOpportunity);
        //VANILLA_SUPPORT(UNTARGET,               AUntarget);
        VANILLA_SUPPORT(WANDER,                 AWander);
        VANILLA_SUPPORT(WEAPONS,                AWeapons);
    case VANILLA_ACTION_TYPE_NONE: return false;
    }

#   undef VANILLA_SUPPORT

    DPRINT(AI, ("Unknown VanillaActionType %u.  No action added.", unsigned(actionType)));
    return false;  // false means "no action added"
}

bool AI::isBigUpdate() const
{
    return zone->isUpdateSubStep(this, kAITimeStep, kAIBigTimeStep);
}

bool AI::isSuperUpdate() const
{
    return zone->isUpdateSubStep(this, kAITimeStep, kAISuperTimeStep);
}

AI::AI(Block* c) : m_config(c)
{
    ASSERT(kAITimeStep > globals.simTimeStep);

    command        = c;
    sc             = command->sb.command.get();
    nav            = new sNav;
    mood           = NEUTRAL;
    moodFaction    = 0;
    target         = NULL;
    priorityTarget = NULL;
    targetPosTime  = 0;
    m_parent       = NULL;

    BlockCluster * cluster = command->cluster;
    ASSERT(cluster);
    if (cluster)
    {
        Faction_t faction = cluster->getFactionActual();
        const FactionData* fac = globals.shipLoader->getFactionData(faction);
        if (fac && fac->ainame.size())
            setMod(fac, false /* don't reset actions */);
    }

    resetForActions();
    recreateActions();
}

AI::~AI()
{

}

const AI* AI::getParentAI() const
{
    return m_parent ? m_parent->getAI() : NULL;
}

void AI::adoptChild(Block* child)
{
    ASSERT(child != command);
    if (!(child->sb.features&Block::GROW))
    {
        if (!child->getAI())
            return;
        ASSERT(child->getAI()->command == child);

        // prevent circular adoption
        const Block *last = NULL;
        for (const Block* ancestor = getParent();
             ancestor && ancestor->commandAI;
             ancestor = ancestor->commandAI->getParent())
        {
            if (ancestor == child ||
                ancestor == command ||
                ancestor == last)
                return;

            if (child->sb.lifetime == -1.f && globals.isOnPlayer(ancestor))
            {
                // ships spawned by player followers belong to player
                removeChild(child);
                ancestor->commandAI->adoptChild(child);
                return;
            }
            last = ancestor;
        }

        AI* cai = child->getAI();
        Block* oldParent = cai->getParent();
        if (oldParent == command) {
            //ASSERT(vec_contains(m_children, watch_ptr<Block>(child)));
            //return;
        } else if (oldParent) {
            cai->removeParent();
        }

        child->sb.command->parentIdent = sc->ident;
        cai->m_parent = command;
        cai->agent = agent;

        if (globals.isOnPlayer(command))
            onPlayerAdopt(child);
    }
    vec_add(m_children, watch_ptr<Block>(child));
}


bool AI::removeChild(Block* child)
{
    if (!vec_remove_one(m_children, child))
        return false;
    //ASSERT(child->getAI()->m_parent == command);
    child->getAI()->m_parent = NULL;
    child->sb.command->parentIdent = 0;
    return true;
}

void AI::removeParent()
{
    if (m_parent)
        vec_remove_one(m_parent->getAI()->m_children, command);
    m_parent = NULL;
    sc->parentIdent = 0;
}


void AI::setTarget(const Block* bl, AIMood mood_)
{
    ASSERT(!bl || bl->isCommand());
    if (bl && bl->isCommand() && bl->commandAI)
        targetPosTime = 0;

    if (!target || target->commandAI)
    {
        target = bl;
        mood   = mood_;
        moodFaction = bl ? bl->getFaction() : 0;
    }
}

const Block* AI::getTarget() const
{
    return or_(priorityTarget.get(),
               target.get(),
               fallbackTarget.get());
}

float2 AI::getTargetPos() const
{
    const Block *tgt = getTarget();
    if (tgt)
        return tgt->cluster->getAbsolutePos();
    else if (canEstimateTargetPos())
        return estimateTargetPos();
    else
        return float2();
}

void AI::clearCommands()
{
    m_actions.clearTag(AIAction::TAG_COMMAND);
}

void AI::appendCommandDest(float2 p, float r)
{
    if (!APath::supportsConfig(m_config))
        return;
    APath *action = new APath(this);
    action->Tag        = AIAction::TAG_COMMAND;
    action->Priority   = PRI_COMMAND;
    action->setPathDest(p, r);
    m_actions.insert(action);
}

void AI::onClusterInit()
{
    m_attackCaps.initialized = false;

    nav->movers.clear();
    foreach (Block* bl, command->cluster->blocks) {
        if (bl->mover)
            nav->movers.push_back(bl->mover.get());
    }

    nav->onMoversChanged();

    sc->resourceCapacity = command->cluster->getResourceCapacity();
}

void AI::onCommandDeath()
{
    // release followers
    foreach (watch_ptr<Block> &bl, m_children)
    {
        if (bl && bl->sb.command) {
            bl->sb.command->flags &= ~SerialCommand::FOLLOWER;
        }
    }

}

Faction_t AI::getFaction() const
{
    return command ? command->getFaction() : 0;
}

// return mobility class:
// 0: immobile
// 1: slow
// 2: fast
int AI::isMobile() const
{
    const float accel = length(nav->maxAccel);
    return accel <= 1.f ? 0 :
        accel < 2.f * command->cluster->getShieldBRadius() ? 1 : 2;
}


void AI::onDamaged(int faction)
{
    if (faction != command->getFaction())
    {
        damagedFaction = faction;
        lastDamagedTime = zone->simTime;
    }
}

void AI::resetForActions()
{
    if (target && !isValidTarget(target.get()))
    {
        target = NULL;
    }
    else if (target && !command->isSensorVisible(target->cluster))
    {
        targetPos     = target->cluster->getAbsolutePos();
        targetVel     = target->cluster->getVel();
        targetPosTime = zone->simTime;
        target        = NULL;
        mood          = AI::NEUTRAL;
        moodFaction   = 0;
    }

    defendPos = float2(0.f);
    signal.reset();
    m_enemiesQueried = false;
    m_alliesQueried  = false;
    m_visibleResources.clear();
    // disable weapons
    foreach (Block *bl, command->cluster->blocks)
        bl->setEnabled(Block::CANNON|Block::LASER|Block::LAUNCH|Block::CHARGING, false);
    nav->dest.dims   = 0;
    zone             = command->cluster->zone;

    if (m_parent && (m_config.flags&ECommandFlags::NO_PARENT))
    {
        removeParent();
        ASSERT(!m_parent);
    }

    if (agent && !agent->command)
    {
        agent->command = command;
        int count = 0;
        foreach (Block* cmd, getAllies())
        {
            if (cmd != command && cmd->commandAI && cmd->commandAI->agent == agent)
            {
                adoptChild(cmd);
                count++;
            }
        }
        DPRINT(AGENT, ("Assuming leadership over %d ships: %s", count,
                       command->summarize().c_str()));
    }
}

void AI::update(bool force)
{
    if (!live(command))
        return;

    // growing/budding children can be in m_children but not have their parent set up
    for (uint i=0; i<m_children.size();) {
        Block *child = m_children[i].get();
        if (!vec_pop_increment(m_children, i, !child)) {
            if (child->commandAI &&
                child->commandAI->getParent() != command)
            {
                adoptChild(child);
            }
        }
    }

    zone = command->cluster->zone;
    if (globals.isOnPlayer(command))
    {
        m_allies.clear();
        signal.reset();
        if (!playerUpdate())
            return;
    }

    command->cluster->getNavConfig(&nav->state);

    if (force || zone->isUpdateStep(this, kAITimeStep))
    {
        recreateActions();

        resetForActions();
        m_actions.update();

        recreateActions();
    }

    nav->update();
}

size_t AI::getSizeof() const
{
    size_t sz = sizeof(AI);
    sz += m_actions.getSizeof();
    sz += (m_enemies.size() + m_allies.size() + m_visibleResources.size() + m_children.size()) * sizeof(void*);
    sz += sizeof(sNav);
    return sz;
}

#include "GUI.h"
#include "Save.h"


void AI::render(DMesh &mesh, float2 screenPos, vector<TextBoxString> &tvec)
{
    if (!(globals.debugRender&DBG_AI))
        return;

    ASSERT_MAIN_THREAD();

    //const bool isPlayer = globals.isOnPlayer(command);

    BlockCluster* cl = command->cluster;

    string desc;
    desc += sc->toPrettyString() + "\n";
    if (m_parent)
        desc += "Parent: " +  m_parent->cluster->summarize() + "\n";
    if (m_children.size()) {
        desc += "Live Children:\n    ";
        const uint count = min(10u, (uint)m_children.size());
        for (uint i=0; i<count; i++) {
            const Block* bl = m_children[i].get();
            if (exist(bl)) {
                desc += bl->cluster->summarize();
                if (i != count - 1)
                    desc += ",\n    ";
            }
        }
        if (count < m_children.size())
            desc += "...";
        desc += "\n";
    }
    // printed as part of serial command
    // str_append_format(desc, "Res: %.f/%.f\n", command->getResources(), cl->getResourceCapacity());

    str_append_format(desc, "Target: %s Env: %.f/%.f = %.2f Mood: %s@%d\n",
                      (exist(target) ? target->cluster->summarize().c_str() :
                       canEstimateTargetPos() ? "<estimated>" : "None"),
                      m_renderEnemyDeadliness, m_renderAllyDeadliness,
                      m_renderEnemyDeadliness / m_renderAllyDeadliness,
                      moodToString(mood), moodFaction);
    if ((zone->simTime - lastDamagedTime) < kAIDamageDefendTime && damagedFaction >= 0)
        str_append_format(desc, "Damaged by %d\n", damagedFaction);
    if (priorityTarget)
        str_append_format(desc, "PriTarget: %s\n", priorityTarget->cluster->summarize().c_str());
    // if (target)
        // str_append_format(desc, "Target: %s\n", target->cluster->summarize().c_str());
    if (fallbackTarget)
        str_append_format(desc, "Fallback Target: %s\n", fallbackTarget->cluster->summarize().c_str());
    if (healTarget)
        str_append_format(desc, "Heal Target: %s\n", healTarget->cluster->summarize().c_str());
    if (signal.type != AISignal::NONE) {
        // FIXME, signal.toString is not thread safe, needs rework anyway
        desc += "Signal: " + string("FIXME") /*signal.toString()*/ + "\n";
    }
    desc += str_format("sizeof ai:%.1fkb command:%.1fkb, cluster:%.1fkb\n",
                       this->getSizeof() / 1024.0,
                       command->getSizeof() / 1024.0,
                       cl->getSizeof() / 1024.0);

    if (m_aiModCreateActions)
    {
        Faction_t faction = cl->getFactionActual();
        const FactionData* fac = globals.shipLoader->getFactionData(faction);
        
        desc += str_format("!! Using AI Mod: \"%s.dll\" from \"%s\"!!\n", fac->ainame.c_str(), fac->name.c_str());
    }
    
    // if (!isPlayer)
        desc += m_actions.toString();

    {
        str_wrap_options_t ops;
        ops.width = 70;
        ops.newline = "\n  ";
        desc = str_word_wrap(desc, ops);
    }

    VertexPusherLine &vp = mesh.line;
    const float2 myPos = command->getAbsolutePos();
    if (live(target))
    {
        vp.color(COLOR_TARGET);
        vp.PushLine(myPos, target->getAbsolutePos());
        vp.PushCircle(target->getClusterPos(), target->cluster->getCoreRadius());
    }
    else if (canEstimateTargetPos())
    {
        vp.color(COLOR_UI_ACTION);
        vp.PushLine(myPos, estimateTargetPos());
    }

    if (live(fallbackTarget))
    {
        vp.color(COLOR_ENEMY);
        vp.PushLine(myPos, fallbackTarget->getAbsolutePos());
        vp.PushCircle(fallbackTarget->getClusterPos(), fallbackTarget->cluster->getCoreRadius());
    }

    if (live(healTarget))
    {
        vp.color(COLOR_PLAYER);
        vp.PushLine(myPos, healTarget->getAbsolutePos());
        vp.PushCircle(healTarget->getClusterPos(), healTarget->cluster->getCoreRadius());
    }

    vp.color(COLOR_C1);
    vp.PushCircle(myPos, cl->getSensorRadius());

    if (defendPos != float2(0))
    {
        vp.color(COLOR_UI_ACTION);
        vp.PushLine(myPos, defendPos);
        vp.PushCircle(defendPos, kComponentWidth);
    }

    if (signal.type != AISignal::NONE)
    {
        vp.color(COLOR_PLAYER);
        vp.PushLine(myPos, signal.pos);
        vp.PushCircle(signal.pos, signal.radius);
    }

    // if (!isPlayer)
    {
        if (nav && !nav->isAtDest())
        {
            vp.color(COLOR_ORANGE);
            if (nav->dest.dims&SN_POSITION) {
                vp.PushLine(myPos, nav->dest.cfg.position);
                vp.PushCircle(nav->dest.cfg.position, nav->precision.pos);
            }
            if (nav->dest.dims&SN_VELOCITY) {
                vp.PushLine(myPos, myPos + nav->dest.cfg.velocity);
            }
            vp.color(COLOR_GREEN);
            if (nav->dest.dims&SN_ANGLE) {
                vp.PushLine(myPos, myPos + cl->getBRadius() * angleToVector(nav->dest.cfg.angle));
            }
        }

        m_actions.render(vp);
    }

    if (cl->getFeatureUnion()&Block::AUTOFIRE)
    {
        vp.color(COLOR_TARGET);
        for (int i=0; i<cl->size(); i++)
        {
            const Block *bl = cl->blocks[i];
            if ((bl->sb.features&Block::AUTOFIRE) && bl->autofireTarget)
            {
                const Block *tgt = bl->autofireTarget.get();
                if (exist(tgt)) {
                    vp.PushLine(bl->getAbsolutePos(), tgt->getAbsolutePos());
                }
            }
        }
    }

    const View &view = globals.getView();
    TextBoxString txt;
    txt.box.tSize = 10.f;
    txt.box.fgColor = ALPHAF(0.8f)|colorIntensify(command->sb.fillColor);
    txt.box.view = &view;
    txt.box.rad = float2(0.7f * view.toScreenSize(command->getClusterBRadius()));
    txt.box.tSize = 10;
    txt.text = str_chomp(std::move(desc));
    txt.position = screenPos;
    tvec.push_back(txt);
}
