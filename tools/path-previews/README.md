# Mayhem pull request previews

Every pull request that changes `src/main/deploy/mayhem/` gets one bot comment,
updated on subsequent pushes. It includes synchronized before/after animations,
an MP4 link, and duration changes for each changed `.mtraj` file. The old route
also appears faintly behind the new one. New and deleted paths are identified.
Videos use Mayhem's actual `FieldArt` and `PlaybackRobot` React/SVG components,
including the project's field decorations, fuel layout, alliance zones, tape,
structure art, collision hatching, speed-colored path and extended intake. They
use Mayhem's light theme and acceleration-based trajectory interpolation. MP4s
are 1920×720; inline GIF previews are 1280×480.
Changes to `project.mayhem` preview all paths because robot dimensions, field
obstacles, and pose variables can change a path without editing its file.

The renderer uses Mayhem solver **0.8.2**. It checks the saved output's input hash
against that side's project and path. Missing, unsuccessful, or stale output is
solved again in an isolated process with a 180-second timeout. Failed new solves
fail the rendering check and are reported instead of playing old output. A failed
baseline solve still permits a preview of a valid new path. Generated output is
never written back to the robot's deployment files.
Fresh saved solutions use their exact samples; no route fitting or replacement
simulation is applied. Regenerated sides are labeled in the comment, since a new
solve can choose a different solution than one previously displayed in the app.
Unsaved desktop edits are not part of the Git diff and cannot appear in previews.

## Enable

Merge both workflow files and this directory into the default branch. The
publisher uses `workflow_run`, so it must exist on the default branch before it
can post comments. Open a PR or push a path change to an existing PR afterward.
No external service, personal access token, or additional secret is required.
Repository Actions policy must permit the declared `contents: write` and
`pull-requests: write` permissions.

Previews are intended for this **public** repository. A dedicated orphan branch,
`codex/path-previews/pr-N` for each PR holds generated GIF, PNG, and MP4 files. Its current tree
keeps only the latest set; older media remains in Git history so earlier
links continue to work. It contains no robot source and should not be merged into
main. Removing the branch removes retained previews after GitHub cache expiry.
Closed PR previews remain available. Large fleets of paths will grow this branch;
the publisher caps each batch at 200 MB and each file at 20 MB.

GitHub's Actions token cannot use the CLI's native video attachment upload.
The comment therefore embeds a looping GIF and links to GitHub's MP4 file viewer.
Full videos and the machine-readable manifest also remain downloadable in the
rendering run's artifacts for 14 days. Videos run at real time up to 45 seconds;
longer trajectories use a labeled playback multiplier. These are trajectory
previews, not full robot/physics/mechanism simulations.

The renderer runs without write permissions or secrets, including for fork PRs.
The publisher only runs default-branch code, validates artifact filenames and
sizes, and checks that the PR still points at the rendered commit before posting.
It treats the downloaded manifest as data, never as Markdown or executable code.

## Local preview

With Python 3.12 or newer and Node.js 22 or newer:

```sh
python3 -m venv /tmp/path-preview-env
/tmp/path-preview-env/bin/pip install -r tools/path-previews/requirements.txt
npm ci --prefix tools/path-previews
npm run build --prefix tools/path-previews
/tmp/path-preview-env/bin/python tools/path-previews/render.py \
  --base origin/main --head HEAD --output /tmp/path-previews
```

The comparison uses the Git merge base, matching the PR diff. A project change
may render many paths and take several minutes. FFmpeg is supplied by the pinned
`imageio-ffmpeg` package; no desktop Mayhem installation is needed. The workflows
have a 60-minute rendering limit and cancel obsolete runs after another push.

Regression checks:

```sh
python -m unittest discover -s tools/path-previews -p 'test_*.py'
node --test tools/path-previews/test_*.cjs
```

The field/playback components, intake geometry, and sampling helper are vendored
from `ryanabraham1/Mayhem` commit `1455950915c621cfee3cf1b4c7fa1046818b4d55`
(`app/src/components/FieldCanvas.tsx`, `app/src/model.ts`, `app/src/styles.css`,
and `app/public/icon.svg`). `mayhem-art.tsx` preserves the component bodies;
`scene.cjs` adds the review layout and rasterizes their SVG with resvg. Since
resvg does not implement non-scaling strokes, it converts those screen widths
to field units before rasterization. No desktop window or browser is required
on the GitHub runner.
