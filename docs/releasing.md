# Release process

`VERSION` is the library's single version source; CMake and archive metadata read
it. guanid is maintained in `RandomPerson5571/guanid` and pinned as the
`libraries/guanid` submodule in VexRobotics. Run release commands from the
guanid repository root. Release tags use `guanid-vVERSION`; a public depot
is not required for local installation.

1. Review pending API/numerical changes and choose a Semantic Versioning version.
   Update VERSION, CHANGELOG, and versioned installation examples together.
2. Run `make test`, then configure/build/CTest with examples enabled. Check both
   supported C++ compiler families and the pinned PROS consumer builds in CI.
3. Run `python3 scripts/package.py --out-dir dist --repository OWNER/REPO` with
   the actual hosting repository. Validate the ZIP with check_package.py and
   install/link it with test_template.py in a fresh PROS project. Review that
   only include/guanid and share/guanid are managed by Conductor.
4. Inspect CHANGELOG, LICENSE/NOTICE, compatibility evidence and limitations.
   Record any physical validation separately; do not silently convert compile
   evidence into a claim of successful hardware trials.
5. Commit the reviewed source, then create/push `guanid-vVERSION`. The repository's
   guanid workflow runs host, package, CMake install, and real PROS consumer jobs.
   Only after those jobs succeed does it create a **draft** GitHub release.
6. Review the draft notes and assets/checksums before publishing. They include
   guanid@VERSION.zip, a complete source ZIP, SHA256SUMS, and a release-specific
   depot JSON. The draft is not a live public release until a maintainer publishes
   it. No release is created just by running local packaging.

The depot JSON is a snapshot for that exact release. For a rolling stable depot,
maintain and host an accumulated version list at a stable URL, retaining old
entries/downloads. Never advertise a URL before it actually exists. A draft asset
is not a usable public install/depot URL.

Reproducible ZIP packaging fixes file order, timestamps and permissions. Rebuilding
with the same source and Python/zlib versions yields identical ZIP bytes. Verify
SHA256SUMS after download using your platform's SHA-256 tool. Generated dist files
are ignored by git; release artifacts are uploaded by CI, not committed.

Do not retag a published release or replace its archive contents. Release a new
patch version for corrections and explain compatibility implications. Protect
release tags and review workflow changes as executable publishing permissions.
