#include "game/StdAfx.h"
#include "AiModExample.h"

#include <game/AI.h>
#include <game/Blocks.h>
#include <game/Sector.h>

#define CVAR_PLACEHOLDER(T, N, V) T N = V;

static CVAR_PLACEHOLDER(int, kAITargetMin, 500);
static CVAR_PLACEHOLDER(float, kAITargetThreshold, 0.25f);
static CVAR_PLACEHOLDER(int, kTeleporterSamples, 20);

#define ADD_ACTION(TYPE, ...)                       \
    if (TYPE::supportsConfig(ai->getConfig()))      \
        ai->addAction(new TYPE(ai, __VA_ARGS__));

#define RUSHRANGEFRAC 0.75			// fraction of bestRange to stay at when rushing
#define SNIPERANGEFRAC 0.95			// fraction of bestRange to stay at when sniping
#define MISSILERANGEFRAC 0.95		// if missiles are main weapon, stay at more than estimated maxRange because game is bad at determining missile range
#define MISSILEDPSFRAC 1.0		// if missile DPS * MISSILEDPSFRAC is more than other DPS, use missiles as main weapon
#define CRITDMGFRAC 0.65		// if ship's current HP is more than 0.65 * max HP, consider ship critically damaged
#define RUSHCVMULT 1.1		// if enemy combatVal * RUSHCVMULT < own combatVal, rush
#define RAMHPMULT 4.0		// if own HP is more than RAMHPMULT * target HP and own DPS is more than RAMDPSMULT * target DPS, ram
#define RAMDPSMULT 2.0

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

/*struct AAvoidWeapon2 final : public AIAction {		// new evasion behaviour

	snConfig      config;

	int           culled = 0;
	int           count = 0;

	static bool supportsConfig(const AICommandConfig& cfg) { return cfg.isMobile; }
	virtual const char* toPrettyString() const { return _("Dodging"); }
	virtual string toStringEx() const { return str_format("Culled %d/%d", culled, count); }

	AAvoidWeapon2(AI* ai) : AIAction(ai, LANE_MOVEMENT, PRI_ALWAYS) { }

	virtual uint update(uint blockedLanes);
};

uint AAvoidWeapon2::update(uint blockedLanes)
{
	// give other actions a chance to run sometimes
	//if (m_ai->isBigUpdate())
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

		const float2 pos = cl->getAbsolutePos();
		const float2 vel = cl->getAbsoluteVel();
		const float  brad = cl->getBRadius();
		const float  radius = cl->command->sb.command->sensorRadius;

		const GameZone *zone = cl->zone;

		const float2 tpos = m_ai->getTargetPos();
		const float  tradius = m_ai->getAttackCaps().bestRange;
		const bool   hasTarget = !nearZero(tpos) && tradius > 0.f;

		float2 bestPos;
		int    bestScore = bestDamage;

		for (int i = 0; i<kTeleporterSamples; i++)
		{
			const float2 spos = pos + randpolar(brad, radius);

			if (zone->intersectPointBounds(spos) ||
				zone->intersectCircleClusterCirclesNearest(spos, brad))
				continue;

			int score = 0;

			foreach(const Obstacle &ob, obs)
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
		uint dims = (nearZero(m_ai->rushDir) ? SN_VELOCITY | SN_VEL_ALLOW_ROTATION : SN_VELOCITY);
		m_ai->nav->setDest(config, dims, 0);
		return LANE_MOVEMENT;
	}
	return LANE_NONE;
}*/

struct AAttack2 final : public APositionBase {

    snConfigDims targetCfg;		// stores target data

	AAttack2(AI* ai): APositionBase(ai) { }

    static bool supportsConfig(const AICommandConfig& cfg);
    virtual uint update(uint blockedLanes);

    virtual const char* toPrettyString() const { return status; }
};



bool AAttack2::supportsConfig(const AICommandConfig& cfg)		// determines which actions get added
{
    return cfg.hasWeapons && (cfg.features&FIREABLE_WEAPONS) && AMove::supportsConfig(cfg);		// checks weapons which can be fired (e.g. exclude melee, contact explosive)
}

uint AAttack2::update(uint blockedLanes)
{
    m_ai->rushDir = float2();
    const Block *target = m_ai->target.get();	// get target - target is a C++ smart pointer 
    if (isTargetObstructed(target))
        return LANE_NONE;

    target->getNavConfig(&targetCfg.cfg);
    snPrecision precision;						// get within certain radius of NavConfig destination
    precision.pos = getWaypointRadius();

    Block*        command = m_ai->command;
    BlockCluster* cluster = command->cluster;

    const AttackCapabilities &caps = m_ai->getAttackCaps();		// get own attack capabilities

    // can't attack without weapons...
    if (!caps.weapons)
        return noAction("No Weapons");

    const AttackCapabilities &tcaps = target->commandAI->getAttackCaps();	// get attack capabilities of target

    const float2 pos        = cluster->getAbsolutePos();	// return position of centre of mass of own ship
    const float2 targetPos  = target->getAbsolutePos();		// return position of centre of mass of enemy ship
    const float2 targetVel  = target->cluster->getVel();	// return target's velocity
    const float  targetDist = distance(targetPos, pos) - 0.5f * target->cluster->getCoreRadius();	// distance between own CoM and target CoM, minus half target's core radius 

    // FIXME targetDist is a hack here... works fine for the common case

    targetCfg.dims = 0;

	const float myPVal = cluster->getDeadliness();
	const float tPVal = target->cluster->getDeadliness();
	const float myBRadius = cluster->getBRadius();
	const float myCRadius = cluster->getCoreRadius();
	const float tBRadius = target->cluster->getBRadius();
	const float tCRadius = target->cluster->getCoreRadius();
    const float myCombatVal = caps.rushDps / tcaps.totalHealth;		// own rushDps divided by target HP
    const float tCombatVal = tcaps.totalDps / caps.totalHealth;		// target rushDps divided by own HP
	const float myMissileDps = caps.totalDps - caps.rushDps;		// totalDps - rushDps = DPS of missile weapons
	const float tMissileDps = tcaps.totalDps - tcaps.rushDps;
	//const float longSnipeRange = LONGSNIPERANGEFRAC * caps.bestRange;		// FIXME implement this
	//const float shortSnipeRange = SNIPERANGEFRAC * caps.bestRange;		// FIXME implement this
    const uint64 flags = m_ai->getConfig().flags;

	//const bool isCriticallyDamaged = (cluster->getHealthFraction() < CRITDMGFRAC);		// ship is considered critically damaged and should disengage if current HP < CRITDMGFRAC - FIXME this causes Attack to be used instead of AAttack2

	const bool isMissileShip = (myMissileDps > MISSILEDPSFRAC * caps.rushDps);		// consider missiles as main weapon if own missile DPS > MISSILEDPSFRAC * own non-missile DPS

	const bool lowTargetValue = (((2.f * tPVal) < myPVal) && (caps.totalHealth < (2.f * tcaps.totalHealth))) ||		// engage more conservatively if target is tougher and lower P
		                        ((tPVal < myPVal) && (caps.totalHealth < (4.f * tcaps.totalHealth))) ||
		                        (5 * tPVal < myPVal);

    const bool rushing = ((myCombatVal > (RUSHCVMULT * tCombatVal)) || // rush if own combatVal is greater than enemy RUSHCVMULT * target's combatVal
		                 (caps.bestRange < 0.8f * tcaps.bestRange)) &&		// rush if own optimal range is much less than target's optimal range
		                 !isMissileShip &&		// don't rush if missiles are main weapon, because missiles spread damage instead of focusing it when fighting at close range
		                 !lowTargetValue ||		// don't rush if engagement has low reward for risk
				         //!isCriticallyDamaged ||	// don't rush if critically damaged
		                 ((flags&SerialCommand::ALWAYS_RUSH) && !(flags&SerialCommand::ALWAYS_MANEUVER) && !(flags&SerialCommand::ALWAYS_KITE));	// rush is now close range engage instead of ram target

	const bool ramming = (caps.rushDps > RAMDPSMULT * tcaps.rushDps) &&
		                 (caps.totalHealth > RAMHPMULT * tcaps.totalHealth) &&
		                 !isMissileShip &&		// don't ram if missiles are main weapon
		                 !(caps.bestRange > tcaps.bestRange) &&		// don't ram if own range is longer than that of target
		                 !lowTargetValue ||		// don't ram if engagement has low reward for risk
		                 ((flags&SerialCommand::ALWAYS_RUSH) && !(flags&SerialCommand::ALWAYS_MANEUVER) && !(flags&SerialCommand::ALWAYS_KITE) && !isMissileShip);	// ram - new behaviour = old rushing

	const bool outgunned = (tCombatVal > 4.f * myCombatVal);		// trigger for more careful engagement behaviour
    const float outrangeRange = 1.1f * tcaps.bestRange;
	const float snipingRange = isMissileShip ? (MISSILERANGEFRAC * caps.maxRange) :		// if missiles are main weapon, stay at longer range since missiles are likely to greatly outrange other weapons
                 		       (SNIPERANGEFRAC * caps.bestRange);		// if enemy is not stronger or you don't have long-range weapons, stay within own bestRange

    const bool canStayOutOfRange = (caps.bestRange > outrangeRange) &&
                                   target->cluster->isMobile() &&
                                   caps.getDpsAtRange(outrangeRange) > tcaps.healthRegen &&
                                   !(flags&SerialCommand::ALWAYS_MANEUVER);

    const bool stayingAtRange = (!rushing && !ramming && canStayOutOfRange) || (flags&SerialCommand::ALWAYS_KITE) || lowTargetValue;
    status = //isCriticallyDamaged ? _("Disengaging") :		// new behaviour for critically damaged ships - run away!
		     ramming ? _("Ramming") :
		     rushing ? _("Rushing") :
		     isMissileShip? _("Bombarding") :		// new behaviour for ships using mostly missiles = engage at missile range instead of relying on bestRange calcs
             stayingAtRange ? gettext_("Sniping") :		// not sure what gettext does, but removing it breaks something
		     _("Manoeuvring");		// changed to British spelling

    const float wantRange = ramming ? 0.f :		// go as close as possible to target if ramming
		rushing ? (RUSHRANGEFRAC * caps.bestRange) :		// stay at RUSHRANGEFRAC of own bestRange if rushing
        snipingRange;			// stay at snipingRange if not rushing


	const float bigTargetExtraRange = ((tCRadius > (0.5f * tBRadius)) &&		// only add extra range if target is round(ish) - FIXME too many magic numbers in this entire initialization
		                              (tBRadius > (0.4f * wantRange))) ? ((0.67f * tBRadius) + (0.4f * wantRange)) :		// if target's BRadius is more than 40% of wantRange, add 67% of target's BRadius plus 40% of wantRange to wantRange
		                              0.f;

	/*const float longShipSubRange = (((1.75f * myCRadius) < (myBRadius)) &&		// only subtract range if own ship's BRadius is more than 1.75 * own CRadius - FIXME too many magic numbers in this initialization too
		                           (myBRadius > (0.6f * wantRange))) ? (0.5f * myBRadius) :		// if own BRadius is more than 80% of wantRange, subtract 50% of own BRadius from wantRange
		                           0.f;
	*/

	const float adjustedWantRange = wantRange + bigTargetExtraRange;		// adjust wantRange to take bigTargetExtraRange and longShipSubRange into account

    const float2 targetLeadPos = targetPos + kAIBigTimeStep * targetVel;	// shoot at position where target will be in (kAIBigTimeStep = 0.5s by default)
    const float2 targetDir = normalize(targetLeadPos - pos);

    /*if (!canStayOutOfRange && wantRange < targetDist)	// charge at target if cannot stay out of range and target is closer than desired range
        m_ai->rushDir = targetDir;
	*/

    const float2 dir = (caps.hasFixed ? directionForFixed(cluster, targetPos, targetVel, FiringFilter()) :		// try to face target if using fixed weapons
                        targetLeadPos - pos);

	const float velocityMultiplier = (ramming || (targetDist > 2.f * adjustedWantRange)) ? 0.9f :		// if ramming (or if target is too far away), match velocity with target closely to get as near to it as possible
		                             isMissileShip ? 1.75f :		// if missiles are main weapon, go much faster than the target to be very careful not to get close
		                             (outgunned || lowTargetValue) ? 1.25f :		// if outgunned or engaging low value target and not ramming, go a little faster than the target to either get closer to retreating targets or prevent being rammed
		                             0.9f;		// if not outgunned, can afford to be less careful

    // move to the optimal attack range
    targetCfg.cfg.position = targetLeadPos - targetDir * adjustedWantRange;		// move to position at desired range from target
    targetCfg.cfg.velocity = velocityMultiplier * targetVel;		// aim to move at different velocity relative to target depending on current behaviour
    targetCfg.cfg.angle = vectorToAngle(dir);		// determine what direction to face
    targetCfg.dims = SN_POSITION | SN_ANGLE | (rushing ? SN_TARGET_VEL : SN_VELOCITY);
    precision.pos = (0.05 * adjustedWantRange);		// movement precision needed - was "max(precision.pos, 0.1f * caps.bestRange)"

    // escape super fast if we are staying at range, bombarding, or AI is set to always kite
    if (stayingAtRange && outgunned || isMissileShip || lowTargetValue || (flags&SerialCommand::ALWAYS_KITE)) {
		if ((targetDist < (0.33f * adjustedWantRange)) && (tCombatVal > myCombatVal)) {
			targetCfg.cfg.velocity += 100.f * (targetCfg.cfg.position - pos);		// if enemy is stronger and way closer than desired, don't bother keeping fixed weapons on target, just run away as fast as possible
		}
        else if (targetDist < (0.8 * adjustedWantRange)) {
            targetCfg.cfg.velocity += 10.f * (targetCfg.cfg.position - pos);
            targetCfg.dims = SN_ANGLE | SN_VELOCITY | SN_VEL_ALLOW_ROTATION;		// more important to keep fixed weapons on target than move optimally in desired direction
        }
        else if (targetDist < 1.1f * caps.maxRange) {
            // don't worry about position, just match velocity
            if (caps.hasFixed)
                targetCfg.dims = SN_ANGLE | SN_VELOCITY;
            else
                targetCfg.dims = SN_ANGLE | SN_VELOCITY | SN_VEL_ALLOW_ROTATION;
        }
    }
    if (caps.hasFixed && targetDist <= caps.maxRange) {
        targetCfg.dims |= SN_POS_ANGLE;		// something to do with pointing in right direction more aggressively with fixed weapons
    }

	if ((targetDist < 0.3f * adjustedWantRange) && rushing) {
		targetCfg.cfg.velocity += 1.2f * (targetCfg.cfg.position - pos);		// if rushing and enemy gets too close, move away slowly
	}

	/*if (isCriticallyDamaged) {
		targetCfg.cfg.velocity += 100.f * (targetCfg.cfg.position - pos);		// if critically damaged just run away as fast as possible
	}*/

    if (!targetCfg.dims)
        return noAction("No direction");

    m_ai->nav->setDest(targetCfg.cfg, targetCfg.dims, precision);		// turn on thrusters
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
		//ADD_ACTION(AAvoidWeapon2);
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