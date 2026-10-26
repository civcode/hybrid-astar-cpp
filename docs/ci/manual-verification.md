# Manual GitHub Actions verification (explicit opt-in only)

The repository has a single [manual C++ verification workflow](../../.github/workflows/manual-verification.yml).
Its **only** event is `workflow_dispatch`; it has no push, pull-request,
schedule, release or chained trigger. The file was created on
`docs/architecture-baseline`, with no workflow runs started.

## Select exactly one suite per run

| Suite | Maximum runner time | Scope | When to choose |
| --- | ---: | --- | --- |
| `quick` **(default)** | 15 min | Ubuntu 24.04, CMake + fetched Catch2 3.16.1, core W10–W12, CLI, demo, map, Node and HTTP CTest; OMPL off | After changes to core or the demo |
| `sanitizers` | 30 min | Same core and demo test build with AddressSanitizer + UndefinedBehaviorSanitizer, sequential CTest; OMPL off | Before a significant merge or after memory-safety changes |
| `ompl` | 60 min | Existing Docker test target: pinned OMPL 2.0.2 compiled from source, fetched Catch2 3.16.1, complete OMPL-enabled CTest | After steering/OMPL changes, before release or milestone sign-off |

Each selection starts **one** job on a hosted Ubuntu runner, not a matrix.
Compilation is restricted to two concurrent build processes, tests use at
most two processes (`sanitizers` is sequential), and every job has a hard
timeout. Starting a new manual run for the **same branch** cancels the older
run, to prevent unnecessary concurrent runner usage. Permissions are limited
to repository contents **read-only**. The workflow doesn't run benchmarks,
deploy, publish releases, upload large artifacts or populate durable caches.
A job timeout is a usage ceiling, **not** a prediction of its actual cost.

A failing test or build fails its job, and the GitHub Actions job logs
retain the relevant compiler errors and CTest assertion details.

## GitHub's default-branch activation requirement

GitHub recognizes manual `workflow_dispatch` workflows only when the
workflow file is also present on the repository's **default branch** (`main`).
Having it **only** on `docs/architecture-baseline` does **not** expose a
working **Run workflow** button. This is a GitHub platform requirement.

To activate without merging unfinished W11/W12 work, create a **small PR
to `main` containing only** `.github/workflows/manual-verification.yml`.
The feature-branch workflow was introduced in commit
[`2a68b1d`](https://github.com/civcode/hybrid-astar-cpp/commit/2a68b1d04a3302e1f14c70f10eafeda072e71348).
Do not merge the whole feature branch just to activate Actions. **No change
to `main` and no workflow run has been made by this setup.**

After that small workflow-only PR is merged, manually open:

1. **Actions** → **Manual C++ verification** → **Run workflow**.
2. Select **`docs/architecture-baseline`** in the branch dropdown.
3. Select `quick` by default; select `sanitizers` or `ompl` only
   when those additional checks are required.
4. Click **Run workflow**. Only this manual step incurs runner usage.

Equivalent explicit command, **after default-branch activation**:

```bash
gh workflow run manual-verification.yml \
  --repo civcode/hybrid-astar-cpp \
  --ref docs/architecture-baseline \
  -f suite=quick
```

To inspect manually requested runs:

```bash
gh run list --repo civcode/hybrid-astar-cpp --workflow manual-verification.yml
```

No GitHub Actions run counts as verified until its actual build and CTest
result are inspected. Start with `quick`; reserve the expensive `ompl`
mode for genuinely needed integration evidence.
