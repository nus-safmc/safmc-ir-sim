# How to review this

Written for you, not for a stranger. It says where to look, what to distrust, what I am least
sure about, and what still needs your decision.

If you have **30 minutes**, do §1 and §5. If you have a day, do all of it.

---

## 1. The thirty-minute pass

In this order. Each step should either satisfy you or produce a question.

**1. Does it run?**

```bash
python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"
.venv/bin/python -m pytest tests -q        # expect: 193 passed, ~37 s
.venv/bin/python examples/01_hello_policy.py
.venv/bin/safmc-run replay runs/hello      # open runs/hello/replay.html
```

Watch the replay. You should see drones fan out, sweep, and a couple land on markers. If that
looks wrong to you, stop there — everything downstream is built on it.

**2. Read [ARCHITECTURE.md](ARCHITECTURE.md).** Four diagrams. If the layering does not match how
you would have split it, that is the most important disagreement we can have, and the cheapest
to fix now.

**3. Read [`src/safmc_sim/api.py`](src/safmc_sim/api.py).** ~300 lines and it is the whole
contract your team lives with. Two commands, one observation. If you would want a third command,
say so now — this is the file that is expensive to change later.

**4. Write a policy.** Ten minutes, from [docs/05-policy-api.md](docs/05-policy-api.md) only.
If you have to open source code to answer something, that is a documentation bug and I want to
know which question it was. A junior-dev agent did exactly this and found six; they are fixed,
but you will find different ones.

**5. Read §5 of this document** — the decisions I could not make for you.

---

## 2. What to be skeptical of

Ordered by how much damage a wrong answer does.

**The numbers, if I ever quote any.** Every result this simulator has produced so far came from
policies that no longer exist. There is currently **no benchmark result you should believe**.
The one shipped policy scores zero by design. Treat any figure in `docs/CHECKPOINTS.md` as
history, not as a claim.

**Assumption A-4, marker detection range.** Defaults to 3.0 m. Nothing in any repo measures it.
It sets how much area a drone sweeps per metre flown, which is the dominant term in every
search comparison this thing will ever produce. **Getting it wrong by 2× could change which
strategy wins.** It is one afternoon with a tag and a tape measure. Do this before trusting any
comparison.

**The arena reading.** Whether the published 2 m minimum gap applies between every pair of walls
changes the layout materially — under the strict reading the search area is close to a 2 m
corridor ring, which favours very different strategies from an open field. I picked the strict
reading. Someone who has seen the real field should overrule me.

**Ground-truth pose.** Every result is conditional on it. It matters most in the Unknown Search
Area, where the rules forbid any navigation aid — real localisation there is dead reckoning.
A strategy tuned here has not been tested in the regime that decides the score.

**My own docstrings.** They are long and they assert things. Where one says "measured" or
"verified", it was; where it explains *why*, that is my reasoning and you should push back on it.

---

## 3. Claims I make, and how to check each one

Do not take these on trust. Each is one command.

| Claim | Check it |
|---|---|
| The raycaster is exact | `pytest tests/test_raycast.py -q` — checked against two independent analytic derivations to 1e-9 m |
| Two identical runs produce identical logs | `pytest tests/test_runner_and_recorder.py -q -k identical` |
| Offline scoring matches online | `pytest tests/test_audit_regressions.py -q -k rescoring` |
| A policy cannot see ground truth | `pytest tests/test_api_and_blackboard.py -q -k ground_truth` — walks every reachable attribute |
| The framework never imports your code | `pytest tests/test_audit_regressions.py -q -k framework` |
| There is no controller in the runner | `pytest tests/test_audit_regressions.py -q -k controller` |
| The arena honours the published gaps | `pytest tests/test_arena.py -q` |

**A test passing is weaker than it looks.** Two of the tests I wrote turned out to be vacuous —
they passed whether or not the code was correct. I found them by *mutation*: break the fix on
purpose and check the test fails. If a test matters to you, try that.

---

## 4. What four blind reviewers found

I ran three blind code reviewers and one junior-dev agent who tried to use the thing from the
README. Full context is in [docs/AUDIT-v0.1.md](docs/AUDIT-v0.1.md) for the earlier spec audit.
The recent round found, and I fixed:

- **You could not run your own policy.** The docs taught a command that could not work.
- **A trap:** drones at different altitudes could not see each other but *could* still collide.
  Altitude looked like a deconfliction axis and was a fleet-killer.
- A second `Runner.run()` silently appended a second fleet and dropped half the result.
- A sub-one-tick run reported a complete-looking, fabricated result.
- `ToFConfig` was applied *after* the ring computed its geometry — the log described a sensor
  that was never simulated.
- `rule_violations` was a metric structurally incapable of being non-zero.

**What I did not fix**, and why:

- The **NED/ROS conversion layer** (~100 lines) is unused by anything in `src/`. Two reviewers
  said cut it. I kept it because portability to ROS/DDS was one of your original requirements,
  and it is cheap and tested. Overrule me if that is no longer the plan.
- The remaining cuts in §5, which need your call first.

---

## 5. Decisions waiting for you

**A. The relay — ~160 lines including tests.** My two reviewers disagreed, so I left it alone.

- *Cut it:* it has never fired in any recorded run, and no shipped policy can reach it. It
  answers a formation question, not a search question.
- *Keep it:* it is worth 2× the entire mission score, which makes it arguably the highest-value
  strategic problem in the competition.

I lean **cut**, because you said not to overfit to the exact competition rules and "how many
targets did you find" is what a strategy comparison needs. Your call — you know the competition.

**B. The fire-suppression coupling — ~45 lines.** Same argument, smaller. It makes target
*ordering* matter, which is a genuine strategic wrinkle.

**C. Is `cruise speed` a target or a cap?** The spec says 0.45 m/s cruise; the code binds it to
a speed *limit*. Those are different models and I could not tell which you meant.

**D. Docs volume.** ~2,900 lines of markdown against ~2,600 of simulator. Some of it is recon
digest (the competition, the hardware, ir-sim's landmines) that is reference material rather
than instructions. Tell me if any of it is noise for your team.

---

## 6. Where the bodies are buried

Things I would look at first if I were trying to break this.

- **`world/arena.py` is the biggest file** (~780 lines). Generation, validation and ir-sim
  emission all live there. It is the most likely place for a subtle bug, and its validation
  once passed a systematic 1.95 m gap in every seed because it never compared the two things
  that were wrong.
- **`mission.py`'s relay BFS** is the most intricate logic in the repo and the least exercised —
  no shipped policy reaches it.
- **The `unobstructed` collision mode** now clamps drones at the field boundary instead of
  crashing them. That keeps it a clean control, but it is a behaviour change worth a look.
- **Determinism depends on nobody calling `numpy.random` directly.** Nothing enforces it; a
  policy that does will break seeded replay silently.
- **Test speed.** Two integration tests dominate the suite. If they get slower, people stop
  running the tests.

---

## 7. If you only do one thing

Measure **A-4** — marker detection range. Everything this simulator will ever tell you about
search strategy is scaled by it, and right now it is a guess.
