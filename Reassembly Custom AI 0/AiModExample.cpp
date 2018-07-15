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

#define RUSHRANGEFRAC 0.4
#define NORMALRANGEFRAC 0.9
#define CAREFULRANGEFRAC 0.9
#define MISSILERANGEFRAC 1.0		// if missiles are main weapon, stay at more than estimated maxRange because game is bad at determining missile range
#define MISSILEDPSFRAC 0.2		// FIXME - change this back to 0.75 (?) after finished testing "Bombarding" behaviour
#define RUSHCVMULT 1.5
#define RAMHPMULT 4.0
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
    const float  targetDist = distance(targetPos, pos) - 0.67f * target->cluster->getCoreRadius();	// distance between own CoM and target CoM, minus 67% target's core radius 

    // FIXME targetDist is a hack here... works fine for the common case

    targetCfg.dims = 0;

    const float myCombatVal = caps.rushDps / tcaps.totalHealth;	// own rushDps divided by target HP
    const float tCombatVal = tcaps.totalDps / caps.totalHealth;	// target rushDps divided by own HP
	const float myMissileDps = caps.totalDps - caps.rushDps; // totalDps - rushDps = DPS of missile weapons
	const float tMissileDps = tcaps.totalDps - tcaps.rushDps;
    const uint64 flags = m_ai->getConfig().flags;

	const bool isMissileShip = (myMissileDps > MISSILEDPSFRAC * caps.rushDps);		//consider missiles as main weapon if own missile DPS > MISSILEDPSFRAC * own non-missile DPS
    const bool rushing = ((myCombatVal > RUSHCVMULT * tCombatVal) ||
		                 (caps.bestRange < 0.8f * tcaps.bestRange) ||		// rush if own optimal range is much less than target's optimal range
		                 (tMissileDps > MISSILEDPSFRAC * tcaps.rushDps)) &&		// rush if target uses missiles as main weapon
		                 !isMissileShip ||		// don't rush if missiles are main weapon, because missiles spread damage instead of focusing it when fighting at close range
		                 ((flags&SerialCommand::ALWAYS_RUSH) && !(flags&SerialCommand::ALWAYS_MANEUVER) && !(flags&SerialCommand::ALWAYS_KITE));	//rush is now close range engage instead of ram target
	const bool ramming = (caps.rushDps > RAMDPSMULT * tcaps.rushDps) &&
		                 (caps.totalHealth > RAMHPMULT * tcaps.totalHealth) &&
		                 !isMissileShip ||		// don't ram if missiles are main weapon
		                 ((flags&SerialCommand::ALWAYS_RUSH) && !(flags&SerialCommand::ALWAYS_MANEUVER) && !(flags&SerialCommand::ALWAYS_KITE) && !isMissileShip);	//ram - new behaviour = old rushing
	const bool outgunned = (tCombatVal > myCombatVal);
    const float outrangeRange = 1.1f * tcaps.maxRange;		// trigger for more careful engagement behaviour
	const float snipingRange = isMissileShip ? (MISSILERANGEFRAC * caps.maxRange) :		// if missiles are main weapon, stay at longer range since missiles are likely to greatly outrange other weapons
		                       outgunned ? (CAREFULRANGEFRAC * caps.maxRange) :		// if enemy is stronger, stay close to own maxRange
		                       (NORMALRANGEFRAC * caps.bestRange);		// if enemy is not stronger, stay just within own bestRange
    const bool canStayOutOfRange = (caps.maxRange >= tcaps.maxRange) &&		// can only stay out of range if own maxRange > enemy maxRange
	                               (caps.maxRange > outrangeRange) &&
                                   target->cluster->isMobile() &&
                                   caps.getDpsAtRange(outrangeRange) > tcaps.healthRegen &&
                                   !(flags&SerialCommand::ALWAYS_MANEUVER);
    const bool sniping = (!rushing && canStayOutOfRange) || (flags&SerialCommand::ALWAYS_KITE);
    status = ramming ? _("Ramming") :
		     rushing ? _("Rushing") :
		     isMissileShip? _("Bombarding") :		// new behaviour for ships using mostly missiles = engage at missile range instead of relying on bestRange calcs
             canStayOutOfRange ? gettext_("Kiting", "Sniping") :		// not sure what gettext does, but removing it breaks something
		     _("Manoeuvring");		// changed to British spelling to check whether displayed status is this

    const float wantRange = ramming ? 0.f :		// go as close as possible to target if ramming
		rushing ? (RUSHRANGEFRAC * caps.bestRange) :		// stay at RUSHRANGEFRAC of own bestRange if rushing
        canStayOutOfRange ? outrangeRange :			// if able to outrange, stay at outrangeRange
        snipingRange;			// stay at snipingRange if not able to stay out of range

    const float2 targetLeadPos = targetPos + kAIBigTimeStep * targetVel;	// shoot at position where target will be in (kAIBigTimeStep = 0.5s by default)
    const float2 targetDir = normalize(targetLeadPos - pos);

    if (!canStayOutOfRange && wantRange < targetDist)	// charge at target if cannot stay out of range and target is closer than desired range
        m_ai->rushDir = targetDir;

    const float2 dir = (caps.hasFixed ? directionForFixed(cluster, targetPos, targetVel, FiringFilter()) :		// try to face target if using fixed weapons
                        targetLeadPos - pos);

	const float velocityMultiplier = ramming ? 0.95f :		// if ramming, match velocity with target very closely to get as near to it as possible
		                             rushing ? 1.05f :		// if rushing, go a bit faster than the target to either avoid being rammed or close distance with fleeing target more quickly
		                             (targetDist > 2.f * wantRange) ? 0.95f :		// if target is at more than twice the desired range, match velocity to close distance
		                             3.f;					// if neither ramming nor rushing, go much faster than the target to be very careful not to get close

    // move to the optimal attack range
    targetCfg.cfg.position = targetLeadPos - targetDir * wantRange;		// move to position at desired range from target
    targetCfg.cfg.velocity = velocityMultiplier * targetVel;		// aim to move at different velocity relative to target depending on current behaviour
    targetCfg.cfg.angle = vectorToAngle(dir);		// determine what direction to face
    targetCfg.dims = SN_POSITION | SN_ANGLE | (rushing ? SN_TARGET_VEL : SN_VELOCITY);
    precision.pos = max(precision.pos, 0.1f * caps.bestRange);		// precision needed is a 1/10 of bestRange

    // escape super fast if we are sniping, bombarding, or AI is set to always kite
    if ((sniping || isMissileShip || (flags&SerialCommand::ALWAYS_KITE)) && !(flags&SerialCommand::ALWAYS_RUSH)) {
		if ((targetDist < 0.75f * wantRange) && (outgunned)) {
			targetCfg.cfg.velocity += 100.f * (targetCfg.cfg.position - pos);		// if enemy is stronger and way closer than desired, don't bother keeping fixed weapons on target, just run as fast as possible
		}
        else if (targetDist < wantRange) {
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
    else if (caps.hasFixed && targetDist <= caps.maxRange) {
        targetCfg.dims |= SN_POS_ANGLE;		// something to do with pointing in right direction more aggressively with fixed weapons
    }

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