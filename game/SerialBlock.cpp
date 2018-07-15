
#include "StdAfx.h"
#include "Blocks.h"

// @Owlfeathers
// With regard to specific adjustments to the automatic P calculations, I think the most
// significant changes I'd suggest are as follows:

// - Heavily reducing the P of spinal-mounted weapons which rely on DPS instead of burst damage
// - (e.g. plasma projector, proton beam), which are usually terrible since the entire ship has
// - to continually remain pointed at the target (and moreover, the same exact spot on the
// - target) to be effective

// - Reducing the value placed on rate of fire for launchers (rapid-fire launchers aren't
// - generally as strong as their P values would suggest - since missiles fired at different
// - times tend to hit different parts of a target, they don't focus damage well). The rapid-fire
// - launchers (with the possible exception of the Terran rocket launcher) are pretty over-valued
// - currently, while even ludicrously powerful launchers like the Nuclear Option aren't all that
// - expensive due to their low rate of fire.

// - Changing drone P calculations - no idea what the formula for this is currently, but most
// - drones are pretty bad currently since huge numbers of drones can be countered with minimal
// - investment in point defence, and even ignoring that, they spread damage all across a target
// - rather than focusing it. Terran drones in particular are also too slow to actually catch any
// - ship frail enough for them to stand a chance at harming, while Tinkrell rocket drones are
// - just horrifically overpriced.

// -  Reducing the value placed on lasers with the CHARGING attribute - these are universally a
// -  bit over-valued, with the possible exception of the Terran burst laser, which finds some
// -  utility as an expensive-but-good point defence system.

// - Including the energy cost of weapons in P calculations - this is a pretty significant factor
// - in a weapon's usefulness which is entirely overlooked by P calculations at the moment.

// - Fixing P calculations for pulsed non-charging lasers. I don't know how this is currently
// - calculated, but there doesn't seem to be any sound logic to it.


// Gamindustri's Lost Soul - Yesterday at 2:14 AM
//As an added note to Owl's list of changes to P calculations I would like to mention that lasers
//entirely ignore explosive properties in calculations. As being explosive can significantly
//increase a laser's power I feel that it is a property that P calculations need to reflect.

// guess at the utility of the block
static float computeBlockDeadliness1(const SerialBlock& sb)
{
    const Feature_t features = sb.getFutureFeatures();
    const float encounterTime = 15;
    float points = 0;

    if (sb.durability > 4.f)
        points += sb.durability / 10.f * max(100.f, sb.getArea());
    else
        points += sb.durability * 1000.f / max(100.f, sb.getArea());

    if (features&Block::INVULNERABLE)
        points += 1000000;
    
    if (features&Block::CANNON) {
        const float roundsPerSec = max(1.f/5.f, (sb.features&Block::CHARGING) ? (1.f / sb.chargeMaxTime) : sb.cannon->roundsPerSec);
        points += encounterTime * min(400.f, sb.cannon->damage) * roundsPerSec *
				  (min(3000.f, sb.cannon->range) / 1200.f) *
				  clamp(sb.cannon->muzzleVel / 1400.f, 0.5f, 2.f) *
                  (sb.cannon->explosive ? 2.f * max(1.f, sb.cannon->explodeRadius / kComponentWidth) : 1.f) *
                  ((features&Block::TURRET) ? 1.f : 0.5f);
    }

    if (features&(Block::LASER)) {
        float rng_pts = (min(1500.f, sb.laser->range) / 1000.f);
        if (sb.laser->range < 500)
            rng_pts /= 3.f;
        if (sb.laser->pulsesPerSec > 0.f)
            rng_pts *= sb.laser->pulseAvailability;
        if (sb.isTurret())
            rng_pts *= 3;
        if (features&Block::CHARGING)
            rng_pts *= 2;
        points += rng_pts * max(encounterTime * abs(sb.laser->damage),
                                3.f * sqrt(max((float)sb.laser->immobilizeForce, sb.laser->linearForce)));
    }

    if (features&Block::CANNON_BOOST) {
        points += encounterTime * 0.5f * sb.boost->damage.y;
        points += encounterTime * 20 * sb.boost->roundsPerSec.x;
        points += encounterTime * (sb.boost->range.y / 100) * (sb.boost->muzzleVel.y / 100.f);
    }

    if (features&Block::EXPLODE)
        points += sb.explodeDamage * max(1.f, sb.explodeRadius / kComponentWidth);

    if (features&Block::MELEE)
        points += 40.f * sb.meleeDamage;

    if (features&Block::LAUNCHER) {
        float missileDeadliness = computeBlockDeadliness1(*sb.replicateBlock);
        if ((sb.replicateBlock->features&Block::COMMAND))
            missileDeadliness *= 1.5f;
        else
            missileDeadliness *= 0.5f;
        points += encounterTime * missileDeadliness / min(5.f, (float)sb.replicateTime);
    }

    if (features&Block::SHIELD) {
        points += (encounterTime * sb.shield->regen + sb.shield->strength) * (sb.shield->radius / 50.f);
    }

    if (features&Block::THRUSTER) {
        points += 2.5f * sqrt(sb.thrusterForce * (max(1.f, (float)sb.thrusterBoost) / 2.f));
    }

    if (features&Block::GENERATOR) {
        if (!(features&Block::COMMAND)) {
            points += 30.f * sb.generatorCapacityPerSec;
        }
    }

    if (features&Block::FACTORY) {
        points += 10000.f;
    }

    if (features&Block::PHOTOSYNTH) {
        points += sb.photosynthPerSec * 1000;
    }

    if (sb.lifetime == -1.f)
    {
        if (features&Block::COMMAND)
            points += 500.f;

        if (features&Block::TRACTOR)
            points += 500.f;
    }

    return points;
}

int SerialBlock::deadliness() const
{
    //const float frac = (health / getMaxHealth());
    // return max(0, round_int(computeBlockDeadliness1(*this) / 100.f));
    return points;
}

int SerialBlock::getPlayerEnergy() const
{
    return (features&(Block::GROW|Block::LAUNCH)) ? 0.f : deadliness();
}

SerialBlock* SerialBlock::finalize()
{
    initFeatures();
    
    if (replicateBlock) {
        replicateBlock->group = group;
        replicateBlock->finalize();
    }

    if (command)
        command->finalize();

    if (cannon && cannon->power < epsilon && cannon->roundsPerSec) {
        cannon->power = round(((cannon->muzzleVel / 1000.f) +
                               (cannon->damage / 5.f) +
                               (cannon->range / 300.f)) / sqrt(cannon->roundsPerSec / 4.f));
    }

    if (features&(Block::LASER) && laser->power < epsilon) {
        laser->power = round((abs(laser->damage)/40.f) * (laser->range/300.f));
    }

    if (shield && shield->color == 0)
    {
        shield->color        = PremultiplyAlphaXXX(colorChangeValue(fillColor, 0.2f), 0.2f, 0.1f);
        shield->lineColor    = PremultiplyAlphaXXX(colorChangeValue(fillColor, 0.2f), 0.2f, 0.05f);
        shield->damagedColor = PremultiplyAlphaXXX(fillColor, 0.7f, 0.3f);
    }
    if (growRate == SerialBlock::getDefault().growRate)
    {
        if (shield) {
            growRate /= 4.f;        // shields regrow extra slowly
        } else if (!(features&(~ISOBLOCK_FEATURE_MASK)) && !isTypeDefault()) {
            // avoid setting non-default value on completely default block
            growRate *= 5.f;
        }
    }
    

    if (generatorCapacityPerSec > epsilon && powerCapacity == 0.f) {
        powerCapacity = 3.f * generatorCapacityPerSec;
    }

    if ((features&Block::COMMAND) && capacity == 0) {
        capacity = 100;
    } else if (features&(Block::TRACTOR|Block::PHOTOSYNTH|Block::GENERATOR) && capacity == 0) {
        capacity = 100;
    }
    if (features&Block::THRUSTER) {
        capacity = max((float)capacity, 0.1f);        // this is kind of a hack for missiles...
        if ((ALPHA_OPAQUE&thrusterColor) == 0)
            thrusterColor |= 0x90000000;
        if (!thrusterColor1)
            thrusterColor1 = colorChangeSaturation(thrusterColor, -0.5);
        if ((ALPHA_OPAQUE&thrusterColor1) == 0)
            thrusterColor1 |= 0x90000000;
    }

    if (!fillColor1 && fillColor != SerialBlock::getDefault().fillColor) {
        fillColor1 = lerpXXX(fillColor, lineColor, 0.4f);
    }

    if (durability == SerialBlock::getDefault().durability)
    {
        if (features&(~ISOBLOCK_FEATURE_MASK))
        {
            durability = 0.5f;
        }
    }

    if (density == SerialBlock::getDefault().density)
    {
        if (features&(~ISOBLOCK_FEATURE_MASK))
        {
            density = 1.5f * kComponentDensity;
        }
    }

    health = getMaxHealth();
    if (!points)
        points = max(0, round_int(computeBlockDeadliness1(*this) / 100.f));

    DASSERT(health > 0.f);
    DASSERT(getMass() > 0.f);
    DASSERT(!cannon || !cannon->explosive || cannon->explodeRadius > 0.f);
    DASSERT(!laser || !laser->explosive || laser->explodeRadius > 0.f);

    return this;
}


float SerialBlock::weaponDamagePerSec() const
{
    Feature_t ftrs = getFutureFeatures();
    float dmg = 0;
    if (ftrs&Block::CANNON) {
        dmg += cannon->damage * cannon->roundsPerSec;
    }
    if (ftrs&(Block::LASER)) {
        dmg += laser->damage;
    }
    if (ftrs&Block::EXPLODE) {
        dmg += explodeDamage;
    }
    if (ftrs&Block::LAUNCHER) {
        dmg += (1.f / replicateTime) * replicateBlock->weaponDamagePerSec();
    }
    if (ftrs&Block::MELEE) {
        dmg += 10.f * meleeDamage;
    }
    return dmg;
}

float SerialBlock::weaponRange(Feature_t ftrs) const
{
    ftrs |= getFutureFeatures();
    float range = 0;
    if (ftrs&Block::CANNON) {
        range = max(range, cannon->range);
    }
    if (ftrs&(Block::LASER)) {
        range = max(range, laser->range);
    } 
    if (ftrs&Block::EXPLODE) {
        range = max(range, (float)explodeRadius);
    }
    if (ftrs&Block::MELEE) {
        range = max(range, spec().radius);
    }
    if (ftrs&Block::LAUNCH) {
        float t = max(lifetime, launchLifetime);
        if (t < 0.f)
            t = 30.f;
        const float t2 = (pow(kPhysicsDamping, t) - 1.f) / logf(kPhysicsDamping);
        range = t2 * launchSpeed;

        if (ftrs&Block::THRUSTER) {
            // FIXME take kPhysicsDamping into account properly...
            const float a = (thrusterForce / (getMass() > 0.f ? getMass() : 5));
            range += 0.5f * a * t;
        } else {
            
        }
    }
    if (ftrs&Block::LAUNCHER) {
        range += replicateBlock->weaponRange(Block::LAUNCH);
        // FIXME launcherOutSpeed?
    }
    return range;
}

float SerialBlock::weaponEfficiency() const 
{
    float powerPerSec = 0.f;
    if (features&Block::CANNON)
        powerPerSec += cannon->power * cannon->roundsPerSec;
    if (features&(Block::LASER))
        powerPerSec += laser->power;
    // LAUNCHER/LAUNCH doesn't really know?
    if (powerPerSec == 0.f)
        return 0.f;
    return abs(weaponDamagePerSec()) / powerPerSec;
}

float SerialBlock::weaponVel() const
{
    if (features&Block::CANNON)
        return cannon->muzzleVel;
    if (features&Block::LAUNCH) {
         const float a = (thrusterForce / (getMass() > 0 ? getMass() : 5));
         return a;              // estimated!
    }
    if (features&Block::LAUNCHER)
        return replicateBlock->weaponVel();
    return 99999999.f;          // lasers
}

bool SerialBlock::weaponSpreads() const
{
    return (features&Block::TURRET) &&
        (features&Block::CANNON) &&
        !(features&Block::AUTOFIRE);
}

bool SerialBlock::isFireable() const
{
    return (features&(Block::LASER|Block::CANNON|Block::LAUNCH|Block::LAUNCHER));
}

bool SerialBlock::isFireableFixed() const
{
    return (((features&(Block::LASER|Block::CANNON)) && !(features&Block::TURRET)) ||
            ((features&Block::LAUNCH) && !(launchFeatures&Block::COMMAND)));
}

bool SerialBlock::isWeapon() const
{
    // launcher are weapons if they launch weapons
    // pure immobilizer lasers are considered weapons
    return (((features&WEAPON_FEATURES) && (!laser || laser->damage >= 0)) ||
            // bibi: ;llllllllllllllllllp`1``````````````````````
            ((features&Block::LAUNCH) && (launchFeatures&~Block::SEED)) ||
            (replicateBlock && replicateBlock->isWeapon()));
}

bool SerialBlock::isHealer() const
{
    return ((features&Block::LASER) && laser && laser->damage < 0) ||
        (replicateBlock && replicateBlock->isHealer());
}

bool SerialBlock::isTurret() const 
{
    return features&Block::TURRET;
}

bool SerialBlock::isTransient() const
{
    // plants have lifetime but need to grow anyway - must not be transient
    return ((getFutureFeatures()&(Block::LAUNCH|Block::AUTOLAUNCH|Block::TRANSIENT)) ||
            //lifetime > 0.f ||
            isTempBlockId(ident));
}

void SerialBlock::setMaxHealth(float mh)
{
    ASSERT(mh > epsilon);
    ASSERT(getArea() > epsilon);
    durability = mh / getArea();
    ASSERT(durability > epsilon);
    ASSERT(getMaxHealth() > 0.f);
    health = mh;
}

void SerialBlock::setGrowTime(float time)
{
    growRate = spec().sqrt_area / time;
    growFrac = 0.f;
}

SerialBlock &SerialBlock::setType(const SerialBlock& sb)
{
    SERIAL_BLOCK_TYPE_FIELDS(SERIAL_COPY_FIELD);
    return *this;
}


#define COPY_FIELD_NON_DEF(TYPE, NAME, DEFAULT) if ( !((sb.NAME) == (DEFAULT))) NAME = sb.NAME;

void SerialCommand::setNonDef(const SerialCommand &sb)
{
    SERIAL_COMMAND_FIELDS(COPY_FIELD_NON_DEF);
}

SerialBlock &SerialBlock::setNonDef(const SerialBlock &sb)
{
    SERIAL_BLOCK_TYPE_FIELDS(COPY_FIELD_NON_DEF);
    return *this;
}

#define IS_NON_DEF(TYPE, NAME, DEFAULT) if ( !((NAME) == (DEFAULT))) return false;

bool SerialBlock::isTypeDefault() const
{
    SERIAL_BLOCK_TYPE_FIELDS(IS_NON_DEF);
    return true;
}


#define COPY_COLOR(X) this->X = sb.X;

SerialBlock &SerialBlock::setColors(const SerialBlock& sb)
{
    initFeatures();
    COPY_COLOR(fillColor);
    COPY_COLOR(fillColor1);
    COPY_COLOR(lineColor);
    COPY_COLOR(thrusterColor);
    COPY_COLOR(thrusterColor1);
    if (cannon && sb.cannon) {
        COPY_COLOR(cannon->color);
    }
    if (laser && sb.laser) {
        COPY_COLOR(laser->color);
    }
    if (shield && sb.shield) {
        COPY_COLOR(shield->color);
        COPY_COLOR(shield->lineColor);
        COPY_COLOR(shield->damagedColor);
    }
    if (replicateBlock && sb.replicateBlock)
        replicateBlock->setColors(*sb.replicateBlock);
    return *this;
}

SerialBlock &SerialBlock::replaceWith(const SerialBlock& sb)
{
    ASSERT(sb.shape == shape);
    ASSERT(sb.scale == scale);
    setType(sb);
    ident   = sb.ident;
    command = sb.command;
    return *this;
}

#define COPY_FIELD_DEF(TYPE, NAME, DEFAULT) if ( ((NAME) == (DEFAULT))) NAME = sb.NAME;

SerialBlock &SerialBlock::updateType(const SerialBlock& sb)
{
    SERIAL_BLOCK_TYPE_FIELDS(COPY_FIELD_DEF);
    return *this;
}

#define INIT_PTR(F, X, T) ((ftrs&(F)) ? ((X) ? (X).get() : (proto.X ? new T(*proto.X) : new T())) : NULL)

void SerialBlock::initFeatures(Feature_t ftrs)
{
    if (!ftrs)
        ftrs = getFutureFeatures();

    const SerialBlock *def = SerialBlock::fromId(ident);
    const SerialBlock &proto = def ? *def : SerialBlock::getDefault();

    cannon.reset(INIT_PTR(Block::CANNON, cannon, SerialCannon));
    replicateBlock.reset(INIT_PTR(Block::LAUNCHER, replicateBlock, SerialBlock));
    boost.reset(INIT_PTR(Block::CANNON_BOOST, boost, SerialCannonBoost));
    command.reset(INIT_PTR(Block::COMMAND|Block::SEED, command, SerialCommand));
    laser.reset(INIT_PTR(Block::LASER, laser, SerialLaser));
    shield.reset(INIT_PTR(Block::SHIELD, shield, SerialShield));
}

Feature_t SerialBlock::getFutureFeatures() const
{
    Feature_t ftrs = features.get();
    if (ftrs&(Block::GROW|Block::UNGROW))
        ftrs |= growFeatures.get();
    if (ftrs&(Block::LAUNCH|Block::AUTOLAUNCH))
        ftrs |= launchFeatures.get();
    return ftrs;
}

void SerialBlock::setGrowLaunch(Feature_t growFtrs, Feature_t launchFtrs, float res)
{
    ASSERT(growFtrs&Block::GROW);
    ASSERT(launchFtrs&(Block::LAUNCH|Block::AUTOLAUNCH));
    ASSERT(res == 0 || (features&Block::COMMAND));
    //ASSERT((getFutureFeatures()&Block::FREERES) || res <= capacity);
	launchFeatures  = features&~(Block::LAUNCH | Block::AUTOLAUNCH);
    launchResources = res;
    launchLifetime  = lifetime;
    growFeatures    = launchFtrs|(features&(Block::MELEE|Block::ENVIRONMENTAL));
    growFrac        = 0.f;
    capacity        = 0.f;
    lifetime        = -1.f;
    features        = growFtrs;
    initFeatures();
}

void SerialBlock::setFaction(Faction_t faction)
{
    if (command)
        command->faction = faction;
    else
        explodeFaction = faction;
}

size_t SerialCommand::getSizeof() const
{
    return sizeof(*this) + SIZEOF_REC(blueprint);
}


size_t SerialBlock::getSizeof() const
{
    size_t sz = sizeof(*this);
    sz += SIZEOF_REC(replicateBlock);
    sz += SIZEOF_REC(command);
    sz += SIZEOF_PTR(cannon);
    sz += SIZEOF_PTR(boost);
    sz += SIZEOF_PTR(laser);
    sz += SIZEOF_PTR(shield);
    return sz;
}

BlockCluster *SerialBlock::createCluster() const
{
    Block *bl = new Block(*this);
    return bl->createCluster();
}

static const SerialBlock *fromIdCheck(BlockId_t ident)
{
    const SerialBlock* sb = SerialBlock::fromId(ident);
    if (!sb) {
        DPRINT(DESERIALIZE, ("No block for ident %d", ident));
    }
    return sb;
}

SerialBlock *SerialBlock::overwriteFromId(BlockId_t id_)
{
    if (id_)
        ident = id_;
    if (isTempBlockId(ident))
        return NULL;
    const SerialBlock* sb1 = fromIdCheck(ident);
    if (!sb1)
        return NULL;
    setType(*sb1);
    initFeatures();
    return this;
}

SerialBlock *SerialBlock::overwriteColorsFromId()
{
    const SerialBlock* sb1 = fromIdCheck(ident);
    if (!sb1)
        return NULL;
    setColors(*sb1);
    return this;
}

SerialBlock* SerialBlock::resetFeaturesFromId()
{
    if (isTempBlockId(ident))
        return NULL;
    const SerialBlock* sb1 = fromIdCheck(ident);
    if (!sb1)
        return NULL;
    setFeatures(sb1->features.get());
    return this;
}

Feature_t SerialBlock::getFeaturesFromId() const
{ 
    const SerialBlock* sb1 = SerialBlock::fromId(ident);
    return sb1 ? sb1->features.get() : 0;
}


SerialBlock *SerialBlock::updateById()
{
    if (isTempBlockId(ident))
        return this;
    const SerialBlock* sb1 = SerialBlock::fromId(ident);
    if (!sb1)
        return NULL;
    updateType(*sb1);
    // initFeatures();
    return this;
}

DEFINE_SERIAL_STRUCT(SerialCannon, SERIAL_CANNON_FIELDS);
DEFINE_SERIAL_STRUCT(SerialCannonBoost, SERIAL_BOOST_FIELDS);
DEFINE_SERIAL_STRUCT(SerialLaser, SERIAL_LASER_FIELDS);
DEFINE_SERIAL_STRUCT(SerialShield, SERIAL_SHIELD_FIELDS);
DEFINE_SERIAL_STRUCT(SerialCommand, SERIAL_COMMAND_FIELDS);
DEFINE_SERIAL_STRUCT(ShipData, SERIAL_SHIP_DATA);
DEFINE_SERIAL_STRUCT(SerialBlock, SERIAL_BLOCK_ALL_FIELDS)

bool SerialBlock::isTypeEqual(const SerialBlock& sb) const
{
    if (0); 
        SERIAL_BLOCK_TYPE_FIELDS(SERIAL_ELSE_FIELD_NEQUAL)
    else return 1;
}
