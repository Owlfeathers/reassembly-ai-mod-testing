-- attempting to make vanilla factions use modified AI

-- faction specification file
-- playable==2 means unlocked by default
{
   8  = { name=_("Terran"), playable=2, start="8_interceptor",
          color0=0x113077, color1=0xaaaaaa, aiflags=FORGIVING|WANDER|DODGES|SOCIAL|TRACTOR_TRANSIENT, ainame="TestingAI" },
   2  = { name=_("Farmer"), playable=1, start="2_awesomefighter",
          color0=0x79a82d, color1=0x404040, color2=0x061a65, primaries=3, aiflags=WANDER|SOCIAL|CAUTIOUS|FLOCKING|PEACEFUL|TRACTOR_TRANSIENT|DODGES, ainame="TestingAI" },
   3  = { name=_("Red"), playable=1, start="3_brawler",
          color0=0xe1a71c, color1= 0xa01d10, aiflags=FORGIVING|WANDER|DODGES|RECKLESS|METAMORPHOSIS|BAD_AIM, ainame="TestingAI",
          thrustSFX=1},
   4  = { name=_("Tinkrell"), playable=1,
          color0=0x800000, color1= 0x30203b, aiflags=FORGIVING|WANDER|DODGES|AGGRESSIVE|FLOCKING|TRACTOR_TRANSIENT, ainame="TestingAI" },
   -- Faction 5 is the vegetative plant faction (kPlantFaction)
   6  = { name=_("Borg"), color0=0xffffff, color1=0xeeeeee, aiflags=FORGIVING|WANDER|DODGES, ainame="TestingAI" },
   7  = { name=_("Flies"), color0=0x783201, color1=0xE0A231, aiflags=FORGIVING|WANDER|DODGES, ainame="TestingAI" },
   11 = { name=_("Crystalline"), playable=1,
          color0=0x058060, color1=0x074480, aiflags=FORGIVING|WANDER|DODGES|AGGRESSIVE, thrustSFX=1, ainame="TestingAI" },
   10 = { name=_("Contestant"),
          color0=0x800030, color1=0x303030, aiflags=FORGIVING|WANDER|DODGES|SOCIAL|TRACTOR_TRANSIENT, ainame="TestingAI" },
   12 = { name=_("Bee"), playable=1,
          color0=0xF8BC04, color1=0x514644, aiflags=WANDER|DODGES|AGGRESSIVE|HATES_PLANTS, ainame="TestingAI" },
   13 = { name=_("Spiky Plant"), color0=0x98a606, color1=0x677606, aiflags=FORGIVING|HATES_PLANTS|WANDER|NO_PARENT, ainame="TestingAI" },
   15 = { name=_("Sentinel"), playable=1, start="15_scythe",
          color0=0x96bc2a, color1=0x404840, aiflags=WANDER|DODGES, ainame="TestingAI" },
}
