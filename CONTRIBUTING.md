# Contributing to SRM Autonomous Buggy

**SRM Institute of Science and Technology — Phase 1 SITL Sprint · March 2026**  
Department of Robotics and Automation Engineering · 6-Person Team

This document is the single source of truth for how every team member contributes to this repository. Read it completely before writing a single line of code or creating a branch. Following these rules keeps the codebase clean, keeps `colcon build` green, and makes sure the HOD demo never breaks because of a Git accident.

---

## Table of Contents

- [Who This Applies To](#who-this-applies-to)
- [The Golden Rules](#the-golden-rules)
- [Repository Structure](#repository-structure)
- [Git Workflow — Step by Step](#git-workflow--step-by-step)
- [Branch Naming](#branch-naming)
- [Commit Message Format](#commit-message-format)
- [Pull Request Process](#pull-request-process)
- [Code Review Standards](#code-review-standards)
- [Merging Rules](#merging-rules)
- [What is Banned](#what-is-banned)
- [Python Code Standards](#python-code-standards)
- [ROS 2 Node Standards](#ros-2-node-standards)
- [Testing Before You Push](#testing-before-you-push)
- [Team Ownership Map](#team-ownership-map)
- [Day-by-Day Contribution Checkpoints](#day-by-day-contribution-checkpoints)
- [How to Handle Conflicts](#how-to-handle-conflicts)
- [Emergency Procedures](#emergency-procedures)

---

## Who This Applies To

Every person on all three sub-teams — Alpha, Bravo, and Charlie — must follow this guide. There are no exceptions for "just a small fix" or "I'll clean it up later." The repository is a shared engineering deliverable that represents all six of us in the HOD demo.

---

## The Golden Rules

These five rules are non-negotiable. Everything else in this document explains how to follow them.

1. **Never commit directly to `main`.** Always work on a branch.
2. **Never force push.** Once a commit is pushed, it stays in history.
3. **Never rebase a branch that has been pushed and shared.** Rebase only your own local unpushed work.
4. **Every change goes through a Pull Request with at least one reviewer.**
5. **`colcon build` must pass with zero errors before any PR is opened.**

---

## Repository Structure

Understanding where files live prevents you from accidentally editing the wrong package. The repo is structured exactly as specified in Section 2.4 of the Master Implementation Plan.

```
srm_buggy_ws/
├── src/
│   ├── buggy_description/      ← Team Alpha owns this
│   │   ├── urdf/               ← XACRO files, sensor links
│   │   └── meshes/             ← STL/DAE mesh files
│   │
│   ├── buggy_bringup/          ← Team Alpha owns this
│   │   ├── launch/             ← All launch files
│   │   ├── worlds/             ← srm_campus.world SDF
│   │   └── rviz/               ← RViz2 config file
│   │
│   ├── buggy_brain/            ← Team Bravo + Charlie own this
│   │   └── buggy_brain/
│   │       ├── map_graph.py
│   │       ├── path_planner_node.py
│   │       ├── waypoint_follower.py
│   │       ├── obstacle_detector.py
│   │       ├── ultrasonic_monitor.py
│   │       ├── state_machine.py
│   │       ├── speed_controller.py
│   │       ├── crowd_detector.py
│   │       └── demo_visualizer.py
│   │
│   └── docs/                   ← All teams contribute
│       ├── implementation_plan.pdf
│       ├── traceability_matrix.md
│       └── assets/
│
├── run_demo.sh                 ← Team Charlie owns this
├── .gitignore
└── README.md
```

**The `.gitignore` must always exclude:**

```
build/
install/
log/
__pycache__/
*.pyc
*.pyo
.vscode/
*.swp
```

---

## Git Workflow — Step by Step

Follow this exact sequence every time you start work. Do not skip any step.

### Step 1 — Sync your local `main` before starting

```bash
git checkout main
git pull origin main
```

Always pull first. If you branch off a stale `main`, your PR will have unnecessary conflicts.

### Step 2 — Create your feature branch

```bash
git checkout -b feature/your-feature-name
```

See the Branch Naming section below for exactly how to name it.

### Step 3 — Do your work in small, focused commits

```bash
# After making a logical unit of change:
git add path/to/changed/file.py
git commit -m "feat: add obstacle consecutive-reading filter (§5.1)"
```

Commit often. Small commits are easier to review and easier to revert if something breaks.

### Step 4 — Keep your branch up to date while you work

If other teammates have merged into `main` while you were working, bring those changes into your branch using a merge (not a rebase):

```bash
git checkout main
git pull origin main
git checkout feature/your-feature-name
git merge main
```

Resolve any conflicts, then commit the merge.

### Step 5 — Push your branch

```bash
git push origin feature/your-feature-name
```

This is a normal push. Never use `--force` here.

### Step 6 — Open a Pull Request on GitHub

Go to the GitHub repository, find your branch, and open a Pull Request against `main`. Fill in the PR template completely (see Pull Request Process below).

### Step 7 — Address review feedback

If your reviewer requests changes, make them on the same branch and push again. The PR updates automatically. Do not open a new PR for the same feature.

### Step 8 — Merge after approval

Once the PR is approved and all conversations are resolved, merge using the GitHub UI with "Create a merge commit" selected. Do not use squash merge or rebase merge.

### Step 9 — Clean up

```bash
# Delete the remote branch (GitHub may do this automatically after merge)
git push origin --delete feature/your-feature-name

# Delete the local branch
git checkout main
git pull origin main
git branch -d feature/your-feature-name
```

---

## Branch Naming

Branch names must be lowercase with hyphens. No spaces, no underscores, no uppercase letters.

| Prefix | When to use | Examples |
|--------|-------------|---------|
| `feature/` | Adding a new node, new behaviour, new sensor, new world element | `feature/crowd-detector`, `feature/campus-world-sdf`, `feature/demo-visualizer` |
| `fix/` | Fixing a bug in existing code | `fix/waypoint-oscillation`, `fix/lidar-scan-zeros`, `fix/gazebo-clock-desync` |
| `docs/` | Changes to README, CONTRIBUTING, comments, traceability matrix, or diagrams only — no code changes | `docs/update-traceability`, `docs/add-rqt-screenshot` |
| `test/` | Adding or modifying test scripts, stress test tooling, or verification commands | `test/stress-run-script`, `test/obstacle-trigger-test` |
| `chore/` | Tooling, CI config, `.gitignore`, `run_demo.sh` cleanup — no functional code changes | `chore/update-gitignore`, `chore/fix-demo-script` |

**Bad branch names to avoid:**

```
# These will be rejected or asked to rename before merging
my-branch
Member1-stuff
fix
temp
test123
FEATURE_CROWD
```

---

## Commit Message Format

Every commit message must follow this format:

```
<type>: <short description in present tense> (<plan section if relevant>)
```

The short description should be 50 characters or fewer. Use the imperative present tense — "add", "fix", "update", not "added", "fixed", "updated".

**Allowed types:**

| Type | When to use |
|------|-------------|
| `feat` | Adding new functionality |
| `fix` | Fixing a bug |
| `docs` | Documentation only changes |
| `test` | Adding or fixing tests |
| `chore` | Build tools, scripts, config — no production code |
| `refactor` | Code restructuring that does not change behaviour |

**Good commit message examples:**

```bash
git commit -m "feat: add LiDAR consecutive-reading filter for obstacle detection (§5.1)"
git commit -m "feat: implement 7-state FSM with EMERGENCY_STOP and CROWD_DETECTED (§6.7)"
git commit -m "fix: increase waypoint arrival radius from 0.8 to 1.2 m to stop oscillation"
git commit -m "fix: wrap input() in daemon thread to prevent ROS spin freeze"
git commit -m "feat: add ultrasonic monitor node with 0.30 m threshold (§11.1)"
git commit -m "feat: create srm_campus.world with HUB at (0,0) and three destinations (§5.1)"
git commit -m "chore: add pkill cleanup to run_demo.sh to fix Gazebo crash on 3rd run"
git commit -m "docs: add traceability matrix linking speed_controller to §2.2 ODD"
git commit -m "fix: add use_sim_time:=True to all nodes in full_system.launch.py"
```

**Bad commit message examples — do not do this:**

```bash
git commit -m "changes"
git commit -m "fix stuff"
git commit -m "done"
git commit -m "update files"
git commit -m "WIP"
git commit -m "asdfg"
git commit -m "trying to fix the bug with the thing"
```

---

## Pull Request Process

Every PR must include all of the following before it will be reviewed.

### PR Title

Use the same format as a commit message:

```
feat: add crowd detector node with 40-ray LiDAR threshold (§7.1)
fix: resolve waypoint follower oscillation near destination
```

### PR Description Template

Copy and fill this in every time you open a PR:

```
## What this PR does
<!-- One or two sentences describing the change. What problem does it solve or what feature does it add? -->

## Why this change is needed
<!-- Explain the reason. Link to the plan section if relevant. -->
<!-- Example: "Section 5.1 of the Master Plan requires 3 consecutive readings before triggering obstacle_detected." -->

## How I tested it
<!-- List exactly what you ran to verify this works. -->
<!-- Example: -->
<!-- - ran colcon build — zero errors -->
<!-- - ran ros2 topic pub /obstacle_detected std_msgs/msg/Bool '{data: true}' --once — confirmed EMERGENCY_STOP state -->
<!-- - ran the full demo start to destination B — obstacle stop and resume working -->

## Plan section reference
<!-- Which section(s) of SRM_Autonomous_Buggy_Master_Plan.docx does this implement? -->
<!-- Example: §5.1, §6.4 -->

## Checklist
- [ ] colcon build passes with zero errors
- [ ] No existing topic names or message types changed
- [ ] New thresholds/speeds have a plan section comment in the code
- [ ] Only speed_controller.py publishes to /cmd_vel (if relevant)
- [ ] Branch is up to date with main
```

### Reviewer Assignment

- Every PR must have at least one reviewer assigned
- Assign a teammate from a different sub-team when possible — cross-team review catches more issues
- Tag your reviewer in a comment if they do not respond within a few hours

---

## Code Review Standards

### For the person reviewing

- Read the entire diff, not just the summary
- Check that every new threshold or constant has a comment referencing the plan section
- Verify the PR description mentions how the change was tested — if it does not, ask for it
- Leave specific, actionable comments — "this looks fine" is not a review
- If something is a suggestion and not a blocker, prefix it with `nit:` so the author knows it is optional
- Approve only when you are genuinely satisfied, not just to be polite — your name is on this too

### For the person receiving review

- Respond to every comment, even if just to say you made the change
- Do not resolve conversations yourself — let the reviewer resolve them after they are satisfied
- Do not argue in comments — if you disagree, have a voice conversation and then document the outcome in the PR

---

## Merging Rules

### Always use a regular merge commit

When merging a PR on GitHub, select **"Create a merge commit"**. Do not use:
- Squash and merge
- Rebase and merge

Regular merge commits keep the full history of your branch and make it possible to see exactly when and how a feature was introduced.

### No force pushes — ever

`git push --force` rewrites history. If your branch is already pushed and shared, force pushing will break your teammates' local copies. If you think you need to force push, stop and talk to the team first.

```bash
# This command is banned
git push --force

# This is also banned
git push --force-with-lease
```

The only exception is if you are the only person who has ever touched the branch and it has never been reviewed — even then, prefer not to.

### No rebasing shared branches

Rebasing rewrites commit hashes. If you rebase a branch that your reviewer has already checked out locally, their copy will diverge from yours in a confusing way.

```bash
# Never do this on a branch that has been pushed
git rebase main

# Only allowed on a branch that has NEVER been pushed to origin
git rebase main   # only if branch is 100% local and never pushed
```

If you need to bring `main` into your branch after it has been pushed, use a merge:

```bash
git checkout feature/your-branch
git merge main
# resolve conflicts if any
git commit
git push origin feature/your-branch
```

---

## What is Banned

The following actions are explicitly prohibited in this repository. If you accidentally do one of these, tell the team immediately — it is always fixable if caught early.

| Banned Action | Why it is banned |
|---------------|-----------------|
| `git push --force` on any branch | Rewrites history, breaks teammates' local copies |
| `git push --force-with-lease` on shared branches | Same effect as force push |
| `git rebase` on any branch that has been pushed | Rewrites commit hashes, causes divergence |
| Direct commits to `main` | `main` is the stable branch; unreviewed code breaks the demo |
| Merging your own PR without a reviewer | No self-approval, ever |
| Changing existing ROS 2 topic names without team discussion | Topic names are contracts; changing one silently breaks all subscribers |
| Changing existing ROS 2 message types without team discussion | Same reason |
| Publishing to `/cmd_vel` from any node other than `speed_controller.py` | Creates conflicting velocity commands, unpredictable buggy behaviour |
| Committing `build/`, `install/`, or `log/` directories | These are generated by colcon and must never be in version control |
| Committing `__pycache__/` or `.pyc` files | Same — generated files |

---

## Python Code Standards

All Python nodes in `buggy_brain/` must meet these standards before a PR is opened.

### File header

Every Python node file must begin with a comment block that identifies it:

```python
#!/usr/bin/env python3
"""
obstacle_detector.py
--------------------
Reads /scan at 10 Hz and publishes /obstacle_detected and /obstacle_direction.
Stop threshold: 1.5 m — Master Plan §5.1
Consecutive reading filter: 3 detections to trigger, 5 clears to resume — §5.1
"""
```

### Plan section comments

Every threshold, speed value, state name, or design decision that comes from the Master Implementation Plan must have an inline comment referencing the section:

```python
OBSTACLE_STOP_THRESHOLD = 1.5        # §5.1 — LiDAR emergency stop distance
ULTRASONIC_STOP_THRESHOLD = 0.30     # §11.1 — Defence-in-depth corner guard
MAX_SPEED_MS = 4.17                  # §2.2 ODD — 15 km/h maximum speed
CROWD_RAY_THRESHOLD = 40             # §7.1 / §8.2 — LiDAR ray-density crowd proxy
CONSECUTIVE_DETECT_COUNT = 3         # §5.1 — readings before triggering obstacle
CONSECUTIVE_CLEAR_COUNT = 5          # §5.1 — readings before resuming navigation
WAYPOINT_ARRIVAL_RADIUS = 0.8        # §6.3 — P-controller arrival distance
```

### Threading and `input()`

Never call `input()` in the main thread or in a timer callback. Wrap it in a daemon thread:

```python
import threading

def destination_input_loop(self):
    while rclpy.ok():
        dest = input("Select Destination (A/B/C): ").strip().upper()
        # process input...

# In __init__ or on_activate:
t = threading.Thread(target=self.destination_input_loop, daemon=True)
t.start()
```

### sim_time

Every node must respect simulation time. Always declare and use `use_sim_time` as a parameter:

```python
self.declare_parameter('use_sim_time', False)
```

And always include `use_sim_time:=True` in the launch file for every node.

### No bare `except`

Always catch specific exceptions. Bare `except:` swallows bugs silently.

```python
# Bad
try:
    ranges = msg.ranges
except:
    pass

# Good
try:
    ranges = msg.ranges
except AttributeError as e:
    self.get_logger().error(f"LaserScan message missing ranges field: {e}")
```

---

## ROS 2 Node Standards

### Topic name contracts

The following topic names are fixed and must not be changed by any individual. They are shared contracts between all nodes. Changes require a team discussion and a `chore/` PR that updates all affected nodes together.

| Topic | Publisher | Subscribers |
|-------|-----------|------------|
| `/scan` | Gazebo LiDAR | `obstacle_detector`, `crowd_detector` |
| `/cmd_vel` | `speed_controller` only | Gazebo Diff Drive |
| `/obstacle_detected` | `obstacle_detector` | `state_machine` |
| `/obstacle_direction` | `obstacle_detector` | `demo_visualizer` |
| `/ultrasonic_alert` | `ultrasonic_monitor` | `state_machine` |
| `/crowd_detected` | `crowd_detector` | `state_machine` |
| `/buggy_state` | `state_machine` | `speed_controller` |
| `/planned_path` | `path_planner` | `waypoint_follower`, RViz2 |
| `/navigation_command` | `path_planner` | `state_machine` |
| `/visualization_markers` | `demo_visualizer` | RViz2 |

### The `/cmd_vel` rule

Only `speed_controller.py` publishes to `/cmd_vel`. `waypoint_follower.py` computes the desired velocity internally and passes it to `speed_controller` via the state machine, not by publishing directly. If you are ever tempted to add a `cmd_vel` publisher to any other node, stop and discuss it with the team.

### State machine states

The following state names are fixed strings published on `/buggy_state`. If you need to check the current state, compare against these exact strings. Do not add new states without a team discussion and a PR that updates the state machine, speed controller, and this document together.

```python
WAITING_FOR_DESTINATION = "WAITING_FOR_DESTINATION"
NAVIGATING              = "NAVIGATING"
EMERGENCY_STOP          = "EMERGENCY_STOP"
RESUMING                = "RESUMING"
CROWD_DETECTED          = "CROWD_DETECTED"
MANUAL_CONTROL          = "MANUAL_CONTROL"
DESTINATION_REACHED     = "DESTINATION_REACHED"
```

### Registration in setup.py

Every new node file must be registered in `buggy_brain/setup.py` under `entry_points` before it can be launched:

```python
entry_points={
    'console_scripts': [
        'path_planner_node     = buggy_brain.path_planner_node:main',
        'waypoint_follower     = buggy_brain.waypoint_follower:main',
        'obstacle_detector     = buggy_brain.obstacle_detector:main',
        'ultrasonic_monitor    = buggy_brain.ultrasonic_monitor:main',
        'state_machine         = buggy_brain.state_machine:main',
        'speed_controller      = buggy_brain.speed_controller:main',
        'crowd_detector        = buggy_brain.crowd_detector:main',
        'demo_visualizer       = buggy_brain.demo_visualizer:main',
    ],
},
```

If you add a new node and forget to register it, it will not be launchable and `colcon build` will succeed but the node will not appear in `ros2 run`.

---

## Testing Before You Push

Run every item on this checklist before opening a PR. If any item fails, fix it before pushing.

### Mandatory checklist

```bash
# 1. Build passes with zero errors and zero warnings
cd ~/srm_buggy_ws
colcon build --symlink-install
# Expected: All packages built successfully

# 2. Source the workspace
source install/setup.bash

# 3. Verify Dijkstra correctness (Team Bravo — every time map_graph.py changes)
cd ~/srm_buggy_ws/src/buggy_brain/buggy_brain
python3 map_graph.py
# Expected:
# START → B: ['START', 'HUB', 'B']
# START → C: ['START', 'HUB', 'C']
# START → A: ['START', 'A']

# 4. Verify all sensor topics are alive (Team Alpha — every time XACRO or plugins change)
ros2 topic list | grep -E "scan|camera|imu|gps|ultrasonic|odom"
# Expected: All 8 topics listed

# 5. Verify LiDAR is at correct Hz
ros2 topic hz /scan
# Expected: ~10 Hz

# 6. Verify state machine transitions (Team Bravo/Charlie — when state_machine.py changes)
ros2 topic pub /obstacle_detected std_msgs/msg/Bool '{data: true}' --once
ros2 topic echo /buggy_state --once
# Expected: EMERGENCY_STOP

# 7. Full end-to-end run (all teams — before any PR that touches navigation or launch files)
./run_demo.sh
# Type B, confirm buggy reaches Library Block
# Type C, confirm buggy reaches Admin Block
```

### Day 8 stress test requirement

Before the final HOD demo, the complete system must pass 8 out of 10 consecutive runs. Record your pass/fail results and include them in your Day 8 PR description.

---

## Team Ownership Map

Each file has a primary owner. If you need to change a file owned by another team, talk to them first and make sure they are the reviewer for that PR.

| File / Directory | Primary Owner | Secondary (reviewer) |
|------------------|--------------|----------------------|
| `buggy_description/urdf/` | Team Alpha | Team Bravo |
| `buggy_description/meshes/` | Team Alpha | — |
| `buggy_bringup/launch/` | Team Alpha | Team Charlie |
| `buggy_bringup/worlds/` | Team Alpha | Team Bravo |
| `buggy_bringup/rviz/` | Team Charlie | Team Alpha |
| `buggy_brain/map_graph.py` | Team Bravo | Team Charlie |
| `buggy_brain/path_planner_node.py` | Team Bravo | Team Charlie |
| `buggy_brain/waypoint_follower.py` | Team Bravo | Team Charlie |
| `buggy_brain/state_machine.py` | Team Bravo | Team Charlie |
| `buggy_brain/obstacle_detector.py` | Team Charlie | Team Bravo |
| `buggy_brain/ultrasonic_monitor.py` | Team Charlie | Team Bravo |
| `buggy_brain/crowd_detector.py` | Team Charlie | Team Bravo |
| `buggy_brain/speed_controller.py` | Team Bravo | Team Charlie |
| `buggy_brain/demo_visualizer.py` | Team Charlie | Team Alpha |
| `run_demo.sh` | Team Charlie | All teams |
| `README.md` | Team Bravo | All teams |
| `CONTRIBUTING.md` | Team Bravo | All teams |
| `docs/traceability_matrix.md` | Team Bravo | All teams |

---

## Day-by-Day Contribution Checkpoints

This is what each team should have committed and merged by end of each day. Use this to verify you are on track.

| Day | Team | Expected merged PR |
|-----|------|--------------------|
| 1 | Alpha | Initial repo structure, base URDF spawning in empty Gazebo, `/cmd_vel` and `/odom` confirmed |
| 2 | Alpha | All 8 sensor topics publishing at correct Hz, `srm_campus.world` renders with road network |
| 3 | Bravo | `map_graph.py` Dijkstra standalone tested, `path_planner_node.py` and `waypoint_follower.py` producing `/cmd_vel` output |
| 4 | Charlie | `obstacle_detector.py`, `ultrasonic_monitor.py`, `state_machine.py`, `speed_controller.py`, `crowd_detector.py` all passing isolation tests |
| 5 | All | `full_system.launch.py` working, end-to-end run to B and C completed, result recorded on video |
| 6 | All | Obstacle stop-and-resume in-loop, crowd detection countdown working |
| 7 | Charlie | `demo_visualizer.py` complete, `run_demo.sh` launches everything in one command, RViz2 config saved |
| 8 | All | Stress test results — 8/10 runs pass, all known bugs documented in Known Pitfalls |
| 9 | Bravo | RQT node graph screenshot committed to `docs/`, full traceability matrix complete, README final |
| 10 | All | Repository clean, all PRs merged, no open draft PRs, repo ready for HOD review |

---

## How to Handle Conflicts

Conflicts happen when two people edit the same file at the same time. Here is how to resolve them correctly.

### When you get a conflict during `git merge main`

```bash
# Git will tell you which files conflict:
# CONFLICT (content): Merge conflict in buggy_brain/state_machine.py

# Open the file and find the conflict markers:
# your version
# their version from main

# Decide which version is correct — or combine both if both changes matter.
# Remove all the <<<<<, =======, >>>>>>> markers.
# Save the file.

# Then:
git add buggy_brain/state_machine.py
git commit -m "chore: resolve merge conflict in state_machine.py"
git push origin feature/your-branch
```

### When to ask for help

If the conflict is in a critical shared file like `state_machine.py`, `setup.py`, or `full_system.launch.py`, do not guess. Stop, message the other team, and resolve it together in a call. A wrong merge in these files will break the whole demo.

---

## Emergency Procedures

### `main` is broken and `colcon build` fails

1. Do not panic and do not try to fix it with another commit directly to `main`
2. Identify the last commit on `main` where the build passed (use `git log --oneline`)
3. Open a `fix/` branch from that last good commit
4. Fix the build error there
5. Open a PR, get one quick review, and merge

### A PR was merged that broke the demo

1. Do not revert immediately — first check if a forward fix is faster
2. If the breakage is serious, create a `fix/` branch immediately
3. Communicate to all teammates so nobody builds new features on top of broken `main`

### Someone accidentally pushed to `main` directly

1. Do not force push to remove it — that makes it worse
2. Open a PR from `main` into a temporary `fix/backup-main` branch, review the change there, then proceed normally
3. Update the branch protection rules on GitHub to prevent direct pushes (Settings → Branches → Add protection rule → Require pull request before merging)

---

<div align="center">

*SRM Institute of Science and Technology · Autonomous Electric Buggy · Phase 1 SITL · v1.0 · March 2026*

**When in doubt, ask your teammate. When still in doubt, open a PR and describe what you are unsure about in the description.**

</div>