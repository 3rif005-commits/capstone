# Machinima Scenario — "Intercept"

> Status: **draft, locked for v1 of the video** (2026-06-20). Camera moves and
> details to be adjusted in future passes. This is the storyboard/script only —
> no implementation yet.

## Purpose & format
- **Audience/goal:** *both* — a cinematic hook to grab attention, then explain
  the project's technologies (capstone-grade explainer + showcase).
- **Target length:** short & punchy. Originally ~60–90s, but the 3-entity
  briefing format pushes realistic length to **~2–2.5 min**. Keep each briefing
  tight (~12–15s) to hold it down.
- **Style reference:** war-game / RTS "codex / intel panel" unit briefings
  (Command & Conquer, Wargame, World of Tanks), then a trailer-style cold open,
  then live gameplay.

## The cast (3 entities)
| Entity | Role in game | Project tech it represents |
|---|---|---|
| **Tank** | Static objective at world origin. The prize both drones fight over. | The defended asset / mission objective. |
| **Kamikaze (X3 multirotor)** | Player-flown threat. Takes off from center, dives at the tank. Slow (~7–9 m/s), agile, can hover. | Player teleop, the attacking threat model. |
| **Interceptor (fixed-wing)** | Autonomous defender. Loiters, launches on head-start delay, runs guidance to catch the kamikaze mid-course. Fast (~28 m/s), ~28 m turn radius, cannot hover. | **The core system:** classical guidance (Augmented PN), GPS-style state estimation, fixed-wing kinematics. |

---

## ACT 1 — Entity briefings (cinematic + explanation, fused)
For each entity: a character-matched camera move, then a **war-game info panel**
(HUD overlay) slides in with name + role-in-game + project-tech. Hold, then cut
to the next entity. **Order: Tank → Kamikaze → Interceptor** (prize → threat → hero).

Each entity gets its **own camera language** matched to its character:

### Tank — *"the heavy, the prize"* (imposing, valuable)
- **Crane-down reveal:** start high in the sky, descend + tilt down to settle on
  the tank (god's-eye → ground).
- **Low-angle hero push-in:** finish near ground level looking up, slow dolly-in.
  Panel slides in here.

### Kamikaze (X3) — *"the threat"* (menacing, twitchy, agile)
- **Fast arc + Dutch tilt:** quicker orbit with the horizon deliberately tilted.
- **Buzz-by:** drone makes a close fast fly-by past the lens, then settles. Panel
  slides in on the settle.

### Interceptor (fixed-wing) — *"the hero"* (sleek, fast, lethal)
- **Banking fly-by toward camera + whip-pan:** it screams in low toward the lens,
  camera whip-pans to follow it away (jet hero shot).
- **Tracking/chase shot:** lock alongside as it banks, showing wing + turn
  (visually previews the guidance / turn-radius tech). Panel slides in here.

---

## ACT 2 — Cold open / trailer kill
- Hard cut to the arena. The interceptor **slams the kamikaze at center** before
  it even moves → **explosion**. One brutal hit showing the interceptor's
  lethality. Stylized teaser, NOT the real match.

## ACT 3 — The real duel
- **"Respawn":** kamikaze recovers at center. Now the playable game begins.
- Kamikaze maneuvers toward the **tank**; interceptor flies **Augmented PN
  guidance** to run it down. Maneuvers, near-misses, the catch.

---

## Open / to-adjust later
- Exact camera paths & timings per shot.
- Act 3 outcome framing (clean DEFENSE_WIN vs slow-mo money shot at the catch).
- Whether Act 2 explosion uses a real Gazebo particle effect or an edit overlay.
- Info-panel visual design (font, layout, animation).
- Music / sound design.
