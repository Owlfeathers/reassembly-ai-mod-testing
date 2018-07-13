#include "game/StdAfx.h"
#include "AiModExample.h"

#include <game/AI.h>
#include <game/Blocks.h>
#include <game/Sector.h>

#define CVAR_PLACEHOLDER(T, N, V) T N = V;

static CVAR_PLACEHOLDER(int, kAITargetMin, 500);
static CVAR_PLACEHOLDER(float, kAITargetThreshold, 0.25f);

#define ADD_ACTION(TYPE, ...)                       \
    if (TYPE::supportsConfig(ai->getConfig()))      \
        ai->addAction(new TYPE(ai, __VA_ARGS__));

struct ATargetEnemy2 final : public AIAction {

    typedef std::pair<const Block*, AIMood> Target;
	vector<Target> targets;
    
	ATargetEnemy2(AI* ai) : AIAction(ai, LANE_TARGET) { }

    AIMood acceptTarget(const Block* target) const;
    float targetDistanceMetric(float2 defPos, const Block *tgt) const;
    Target testAcceptTarget(const Block *tgt) const;
    virtual uint update(uint blockedLanes);
};

AIMood ATargetEnemy2::acceptTarget(const Block* target) const
{
    // **** use this to filter targets. return AIMood::NEUTRAL to ignore or AIMood::DEFENSIVE to
    // **** fight back but drop hostilities if target is also defensive/neutral****
    return AIMood::OFFENSIVE;
}

float ATargetEnemy2::targetDistanceMetric(float2 defPos, const Block *tgt) const
{
    // **** Change this to change target priorities ****
    const AttackCapabilities &caps = m_ai->getAttackCaps();
    const float2 tpos = tgt->getAbsolutePos();
    float dist = distanceSqr(defPos, tpos);
    // if we have fixed weapons, weight enemies in front more heavily
    if (caps.hasFixed)
        dist *= 1.f / (2.f + dot(getCluster()->getRot(), normalize(tpos - defPos)));
    return dist;
}

ATargetEnemy2::Target ATargetEnemy2::testAcceptTarget(const Block *tgt) const
{
	if (!tgt || !live(tgt) || !tgt->isCommand())
		return Target();
	const AIMood mood = acceptTarget(tgt);
	if (mood == AIMood::NEUTRAL)
		return Target();
	return make_pair(tgt, mood);
}

uint ATargetEnemy2::update(uint blockedLanes)
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


struct AAttack2 final : public APositionBase {

    snConfigDims targetCfg;

	AAttack2(AI* ai): APositionBase(ai) { }

    static bool supportsConfig(const AICommandConfig& cfg);
    virtual uint update(uint blockedLanes);

    virtual const char* toPrettyString() const { return status; }
};



bool AAttack2::supportsConfig(const AICommandConfig& cfg)
{
    return cfg.hasWeapons && (cfg.features&FIREABLE_WEAPONS) && AMove::supportsConfig(cfg);
}

uint AAttack2::update(uint blockedLanes)
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
		!(flags&(SerialCommand::ALWAYS_MANEUVER| SerialCommand::ALWAYS_KITE));
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


//=============================================================================
// Exported functions
//=============================================================================

void GetApiVersion(int * major, int * minor) {
    *major = 1;
    *minor = 0;
}

// tournament mode AI
bool CreateAiActions(AI* ai) {
    const AICommandConfig &         config = ai->getConfig();
    const ECommandFlags::value_type flags  = config.flags;

    if (config.isMobile >= 2 && (config.flags & SerialCommand::DODGES)) {
        ai->addActionVanilla(VANILLA_ACTION_TYPE_AVOID_WEAPON);
    }

    ai->addActionVanilla(VANILLA_ACTION_TYPE_WEAPONS);

    ai->addActionVanilla(VANILLA_ACTION_TYPE_FALLBACK_TARGET);
    // ai->addActionVanilla(VANILLA_ACTION_TYPE_TARGET_ENEMY);
    ADD_ACTION(ATargetEnemy2);
    ai->addActionVanilla(VANILLA_ACTION_TYPE_AVOID_CLUSTER);
    // ai->addActionVanilla(VANILLA_ACTION_TYPE_ATTACK);
    ADD_ACTION(AAttack2);
    ai->addActionVanilla(VANILLA_ACTION_TYPE_HEALERS); // notice this isn't used by the interceptor, due to supportsConfig()
    ai->addActionVanilla(VANILLA_ACTION_TYPE_INVESTIGATE);

    if (config.features&Block::ASSEMBLER)
    {
        ai->addActionVanilla(VANILLA_ACTION_TYPE_HEAL);
        if (config.flags&SerialCommand::TRACTOR_TRANSIENT) {
            ai->addActionVanilla(VANILLA_ACTION_TYPE_SCAVENGE_WEAPON);
        }
        if (!config.hasFreeRes || kAIEnableNoResReproduce)
        {
            if (config.flags&SerialCommand::METAMORPHOSIS) {
                ai->addActionVanilla(VANILLA_ACTION_TYPE_METAMORPHOSIS);
            }
            ai->addActionVanilla(VANILLA_ACTION_TYPE_BUD_REPRODUCE);
        }
        // else ADonate: find allies and give them resources?
    }
    else if (config.features&Block::REGROWER)
    {
        ai->addActionVanilla(VANILLA_ACTION_TYPE_HEAL);
    }

    if (config.isMobile && config.isRoot() && !config.isAttached)
    {
        ai->addActionVanilla(VANILLA_ACTION_TYPE_PLANT_SELF);
        ai->addActionVanilla(VANILLA_ACTION_TYPE_METAMORPHOSIS);
    }

    if (config.isMobile && !nearZero(ai->command->sb.command->destination))
    {
        ai->appendCommandDest(ai->command->sb.command->destination, 0.25f * kSectorSize);
    }

    if (config.isMobile &&
        !(flags&(SerialCommand::FOLLOWER)) &&
        !config.hasParent &&
        (flags&SerialCommand::WANDER))
    {
        ai->addActionVanilla(VANILLA_ACTION_TYPE_WANDER);
    }

    return true; // we handled it; no need for default AI actions
}
