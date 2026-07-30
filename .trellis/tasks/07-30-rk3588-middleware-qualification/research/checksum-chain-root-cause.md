# Bug Analysis: parent evidence manifest omitted child manifests

## 1. Root Cause Category

- **Category**: D/E — test coverage gap plus implicit assumption.
- **Specific cause**: each generator used
  `find . -type f ! -name SHA256SUMS`. The author intent was to exclude the
  manifest being written, but `-name` excludes every descendant with that
  basename. A qualification parent therefore hashed child evidence files but
  not the child's own deterministic manifest.
- **Confidence**: 99%. Source inspection found the same expression in all
  three platform evidence generators, and the new parent-entry assertion
  distinguishes this cause from a checksum-format or ordering defect.

## 2. Why Earlier Checks Did Not Catch It

1. Host tests asserted that each manifest existed, but never asserted that a
   parent contained the child-manifest path.
2. `sha256sum` generation and ordinary file tamper tests exercised files within
   one level; they did not review the integrity chain across nested bundles.
3. The expression looked like the usual self-exclusion idiom, so review did not
   expand the match scope mentally from the current directory to descendants.

## 3. Prevention Mechanisms

| Priority | Mechanism | Specific action | Status |
|---|---|---|---|
| P0 | Architecture | Exclude only `./SHA256SUMS`; parent manifests include child manifests | DONE |
| P0 | Runtime verification | Recursively verify exact file sets, order and hashes at every manifest | DONE |
| P0 | Integration tests | Assert parent entry and reject a re-anchored invalid child manifest | DONE |
| P1 | Documentation | Record nested-manifest anchoring in the realtime platform contract | DONE |
| P1 | Change propagation | Search all platform generators for the old `! -name SHA256SUMS` form | DONE |

## 4. Systematic Expansion

- **Similar issues**: `collect-evidence.sh`, `qualify-rt.sh` and
  `qualify-middleware.sh` shared the broad exclusion and were all corrected.
  The kernel package builder hashes an explicit flat file list and is not
  affected.
- **Design improvement**: offline verification now treats the bundle as a
  hierarchy of independently deterministic manifests, verifies deepest-first,
  then reports the digest of the top manifest as the external anchor.
- **Process improvement**: evidence tests must inspect manifest membership, not
  merely manifest existence or one successful `sha256sum` invocation.

## 5. Knowledge Capture

- [x] Realtime platform executable contract updated.
- [x] Task design and runbook updated.
- [x] Host and middleware integration regressions added.
- [x] No Trellis template sync applies: this project has no
      `src/templates/markdown/spec/`; the changed spec is project-specific.
