// Runs only from the default branch, with workflow_run's write token.
const fs = require('node:fs');
const path = require('node:path');

const MARKER = '<!-- mayhem-path-previews -->';
const clean = value => String(value).replace(/[\r\n]/g, ' ').replace(/[@<>`*\[\]\\|]/g, '').slice(0, 220);

function readManifest(directory) {
  const manifest = path.join(directory, 'manifest.json');
  if (fs.lstatSync(manifest).isSymbolicLink() || fs.statSync(manifest).size > 2_000_000)
    throw new Error('Invalid preview manifest');
  return JSON.parse(fs.readFileSync(manifest, 'utf8'));
}

function loadReport(directory, head) {
  const report = readManifest(directory);
  if (report.head !== head || !Array.isArray(report.paths) || report.paths.length > 300)
    throw new Error('Preview manifest does not match this run');
  let bytes = 0;
  const files = new Map();
  for (const entry of report.paths) {
    if (!['changed', 'added', 'deleted', 'failed'].includes(entry.status))
      throw new Error('Invalid preview status');
    if (!entry.media) continue;
    if (!/^[0-9a-f]{16}$/.test(entry.media)) throw new Error('Invalid media filename');
    for (const extension of ['gif', 'mp4', 'png']) {
      const name = `${entry.media}.${extension}`;
      const filename = path.join(directory, name);
      const stat = fs.lstatSync(filename);
      if (!stat.isFile() || stat.isSymbolicLink() || stat.size > 20_000_000)
        throw new Error('Invalid preview media');
      bytes += stat.size;
      if (bytes > 200_000_000) throw new Error('Preview media exceeds 200 MB');
      files.set(name, fs.readFileSync(filename));
    }
  }
  return {report, files};
}

async function publish({github, context, core}) {
  const {owner, repo} = context.repo;
  const run = context.payload.workflow_run;
  const api = github.rest;
  let targetSha = run.head_sha;
  let targetNumber;
  if (run.event === 'workflow_dispatch') {
    // Manual runs execute default-branch code only and read PR files as JSON.
    // Never allow a workflow dispatched from an arbitrary branch to select a PR.
    const repository = (await api.repos.get({owner, repo})).data;
    if (run.head_branch !== repository.default_branch || run.head_repository?.full_name !== `${owner}/${repo}`)
      throw new Error('Manual previews must run from the default branch.');
    const manifest = readManifest('path-previews');
    if (!/^[0-9a-f]{40}$/.test(manifest.head) || !Number.isSafeInteger(manifest.prNumber) || manifest.prNumber < 1)
      throw new Error('Invalid manual preview target.');
    targetSha = manifest.head;
    targetNumber = manifest.prNumber;
  }
  const associated = await github.paginate(api.repos.listPullRequestsAssociatedWithCommit,
    {owner, repo, commit_sha: targetSha, per_page: 100});
  const prs = associated.filter(pr => pr.state === 'open' && pr.head.sha === targetSha
    && (!targetNumber || pr.number === targetNumber)
    && pr.base.repo.full_name === `${owner}/${repo}`);
  if (!prs.length) return core.info('No open PR still points at this run.');

  for (const pr of prs) {
    const branch = `codex/path-previews/pr-${pr.number}`;
    // Check current API state again; a newer push may have superseded this run.
    const current = (await api.pulls.get({owner, repo, pull_number: pr.number})).data;
    if (current.head.sha !== targetSha || current.state !== 'open') continue;
    const runUrl = `https://github.com/${owner}/${repo}/actions/runs/${run.id}`;
    let body = `${MARKER}\n### Mayhem path previews\n\nCommit: \`${targetSha.slice(0, 7)}\` · [Workflow and downloadable MP4s](${runUrl})\n\n`;
    let loaded;
    try {
      if (process.env.PREVIEW_DOWNLOAD_OK !== 'true') throw new Error('No preview artifact was produced.');
      loaded = loadReport('path-previews', targetSha);
    } catch (error) {
      core.warning(error.message);
      body += `Preview generation failed. See the workflow for details.\n`;
    }
    if (loaded) {
      const {report, files} = loaded;
      let commit;
      if (files.size) {
        const repository = (await api.repos.get({owner, repo})).data;
        if (repository.private) throw new Error('Inline previews require a public repository.');
        let parent;
        let tree = [];
        try {
          parent = (await api.git.getRef({owner, repo, ref: `heads/${branch}`})).data.object.sha;
        } catch (error) {
          if (error.status !== 404) throw error;
        }
        for (const [name, data] of files) {
          const blob = await api.git.createBlob({owner, repo, content: data.toString('base64'), encoding: 'base64'});
          tree.push({path: `pr-${pr.number}/${name}`, mode: '100644', type: 'blob', sha: blob.data.sha});
        }
        const createdTree = await api.git.createTree({owner, repo, tree});
        const createdCommit = await api.git.createCommit({owner, repo,
          message: `Path previews for PR #${pr.number} at ${targetSha.slice(0, 7)}`,
          tree: createdTree.data.sha, parents: parent ? [parent] : []});
        commit = createdCommit.data.sha;
        if (parent) await api.git.updateRef({owner, repo, ref: `heads/${branch}`, sha: commit});
        else await api.git.createRef({owner, repo, ref: `refs/heads/${branch}`, sha: commit});
      }
      if (!report.paths.length) body += 'No Mayhem paths changed in this diff.\n';
      for (const entry of report.paths) {
        body += `#### ${clean(entry.name)}\n\n`;
        if (entry.status === 'deleted') body += 'Deleted.\n\n';
        else if (entry.status === 'failed') body += '**Could not solve the new path.** See the workflow logs; no stale animation is shown.\n\n';
        else {
          const before = Number(entry.beforeDuration);
          const after = Number(entry.afterDuration);
          if (!Number.isFinite(after)) throw new Error('Invalid duration');
          body += entry.beforeDuration !== undefined && Number.isFinite(before)
            ? `Duration: **${before.toFixed(2)} s → ${after.toFixed(2)} s** (${after-before >= 0 ? '+' : ''}${(after-before).toFixed(2)} s).\n\n`
            : `New path: **${after.toFixed(2)} s**.\n\n`;
          if (entry.beforeError) body += 'Previous path could not be solved; only the new path is playable.\n\n';
          for (const side of ['before', 'after']) {
            if (entry[`${side}Source`] === 'regenerated')
              body += `${side === 'before' ? 'Before' : 'After'}: regenerated with Mayhem 0.8.2 because the saved solution was missing or stale.\n\n`;
          }
          const mediaPath = `${owner}/${repo}/${commit}/pr-${pr.number}/${entry.media}`;
          body += `[![Before and after path playback](https://raw.githubusercontent.com/${mediaPath}.gif)](https://github.com/${owner}/${repo}/blob/${commit}/pr-${pr.number}/${entry.media}.mp4)\n\n`;
          body += `[Watch MP4](https://github.com/${owner}/${repo}/blob/${commit}/pr-${pr.number}/${entry.media}.mp4)\n\n`;
        }
      }
      body += 'Blue-origin field coordinates. Before and after share a clock; the faint route in the after panel is the old path. Playback shows generated trajectories, not robot physics or mechanism actions.\n';
    }
    if (body.length > 60000) body = body.slice(0, 58000) + `\n\nMore previews are available in the [workflow artifacts](${runUrl}).`;
    const comments = await github.paginate(api.issues.listComments,
      {owner, repo, issue_number: pr.number, per_page: 100});
    const comment = comments.find(c => c.user.login === 'github-actions[bot]' && c.body.startsWith(MARKER));
    // Do not publish an obsolete comment if rendering/uploading took a while.
    const latest = (await api.pulls.get({owner, repo, pull_number: pr.number})).data;
    if (latest.head.sha !== targetSha || latest.state !== 'open') continue;
    if (comment) await api.issues.updateComment({owner, repo, comment_id: comment.id, body});
    else await api.issues.createComment({owner, repo, issue_number: pr.number, body});
  }
}

module.exports = publish;
module.exports.loadReport = loadReport;
