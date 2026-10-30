const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const publish = require('./publish.cjs');

test('reject mismatched commits, traversal, and symlink media', () => {
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'preview-test-'));
  const write = report => fs.writeFileSync(path.join(directory, 'manifest.json'), JSON.stringify(report));
  try {
    write({head: 'old', paths: []});
    assert.throws(() => publish.loadReport(directory, 'new'), /does not match/);
    write({head: 'new', paths: [{status: 'changed', media: '../../secret'}]});
    assert.throws(() => publish.loadReport(directory, 'new'), /filename/);
    const media = 'a'.repeat(16);
    write({head: 'new', paths: [{status: 'changed', media}]});
    fs.symlinkSync(path.join(directory, 'manifest.json'), path.join(directory, `${media}.gif`));
    assert.throws(() => publish.loadReport(directory, 'new'), /media/);
  } finally {
    fs.rmSync(directory, {recursive: true});
  }
});

function fixture({head = 'abc1234', existing = false, superseded = false} = {}) {
  const calls = [];
  const pr = {number: 7, state: 'open', head: {sha: head}, base: {repo: {full_name: 'team/robot'}}};
  let gets = 0;
  const associated = () => {};
  const comments = () => {};
  const github = {
    paginate: async method => method === associated ? [pr] : existing
      ? [{id: 42, user: {login: 'github-actions[bot]'}, body: '<!-- mayhem-path-previews -->old'}] : [],
    rest: {
      repos: {listPullRequestsAssociatedWithCommit: associated},
      pulls: {get: async () => ({data: superseded && ++gets > 1
        ? {...pr, head: {sha: 'new-push'}} : pr})},
      issues: {listComments: comments,
        updateComment: async data => calls.push(['update', data]),
        createComment: async data => calls.push(['create', data])},
    },
  };
  return {calls, args: {github, core: {info() {}, warning() {}}, context: {
    repo: {owner: 'team', repo: 'robot'},
    payload: {workflow_run: {head_sha: 'abc1234', id: 123}},
  }}};
}

test('posts a useful error when rendering produced no artifact', async () => {
  process.env.PREVIEW_DOWNLOAD_OK = 'false';
  const {calls, args} = fixture();
  await publish(args);
  assert.equal(calls[0][0], 'create');
  assert.match(calls[0][1].body, /Preview generation failed/);
});

test('updates the existing bot comment instead of adding another', async () => {
  const {calls, args} = fixture({existing: true});
  await publish(args);
  assert.equal(calls[0][0], 'update');
  assert.equal(calls[0][1].comment_id, 42);
});

test('ignores old runs and rechecks for pushes before posting', async () => {
  for (const options of [{head: 'new-push'}, {superseded: true}]) {
    const {calls, args} = fixture(options);
    await publish(args);
    assert.equal(calls.length, 0);
  }
});

test('publishes media to its PR branch and embeds previews in the comment', async () => {
  const cwd = process.cwd();
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'preview-publish-'));
  const media = 'a'.repeat(16);
  const {calls, args} = fixture();
  const gitCalls = [];
  try {
    process.chdir(directory);
    process.env.PREVIEW_DOWNLOAD_OK = 'true';
    fs.mkdirSync('path-previews');
    fs.writeFileSync('path-previews/manifest.json', JSON.stringify({head: 'abc1234', paths: [
      {name: 'DepotExit @everyone', status: 'changed', media, beforeDuration: 4, afterDuration: 3.5},
      {name: 'Removed', status: 'deleted'},
      {name: 'Unsolvable', status: 'failed'},
    ]}));
    for (const ext of ['gif', 'mp4', 'png']) fs.writeFileSync(`path-previews/${media}.${ext}`, 'media');
    args.github.rest.repos.get = async () => ({data: {private: false}});
    args.github.rest.git = {
      getRef: async () => { throw Object.assign(new Error('missing'), {status: 404}); },
      createBlob: async data => {gitCalls.push(data); return {data: {sha: 'blob'}};},
      createTree: async data => {gitCalls.push(data); return {data: {sha: 'tree'}};},
      createCommit: async data => {gitCalls.push(data); return {data: {sha: 'commit'}};},
      createRef: async data => {gitCalls.push(data);},
    };
    await publish(args);
    assert.equal(gitCalls.filter(call => call.encoding === 'base64').length, 3);
    assert.equal(gitCalls.at(-1).ref, 'refs/heads/codex/path-previews/pr-7');
    assert.match(calls[0][1].body, /raw\.githubusercontent\.com\/team\/robot\/commit\/pr-7/);
    assert.match(calls[0][1].body, /4\.00 s → 3\.50 s/);
    assert.match(calls[0][1].body, /Could not solve the new path/);
    assert.doesNotMatch(calls[0][1].body, /@everyone/);
  } finally {
    process.chdir(cwd);
    process.env.PREVIEW_DOWNLOAD_OK = 'false';
    fs.rmSync(directory, {recursive: true});
  }
});

test('manual runs bind the comment to the requested current PR head', async () => {
  const cwd = process.cwd();
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), 'preview-manual-'));
  const head = 'a'.repeat(40);
  const {calls, args} = fixture({head});
  args.context.payload.workflow_run = {event:'workflow_dispatch', head_sha:'b'.repeat(40),
    head_branch:'main', head_repository:{full_name:'team/robot'}, id:123};
  args.github.rest.repos.get = async () => ({data:{default_branch:'main'}});
  try {
    process.chdir(directory);
    process.env.PREVIEW_DOWNLOAD_OK = 'true';
    fs.mkdirSync('path-previews');
    fs.writeFileSync('path-previews/manifest.json', JSON.stringify({head,prNumber:7,paths:[]}));
    await publish(args);
    assert.equal(calls.length,1);
    assert.match(calls[0][1].body,/aaaaaaa/);
    assert.doesNotMatch(calls[0][1].body,/bbbbbbb/);
  } finally {
    process.chdir(cwd);
    process.env.PREVIEW_DOWNLOAD_OK = 'false';
    fs.rmSync(directory,{recursive:true});
  }
});

test('rejects manual publishing from a branch other than main', async () => {
  const {calls,args} = fixture();
  args.context.payload.workflow_run.event = 'workflow_dispatch';
  args.context.payload.workflow_run.head_branch = 'untrusted-branch';
  args.github.rest.repos.get = async () => ({data:{default_branch:'main'}});
  await assert.rejects(publish(args),/default branch/);
  assert.equal(calls.length,0);
});
