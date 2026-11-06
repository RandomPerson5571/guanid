# Release process

`VERSION` supplies the standalone guanid version. Release tags use
`guanid-vVERSION` in `RandomPerson5571/guanid`. A tag publishes a real release
after all required checks pass; it is not a draft-only workflow.

1. Update VERSION, CHANGELOG and the versioned installation/download examples.
   Use a new minor version for breaking pre-1.0 API or output-contract changes.
   Never reuse a published version or tag.
2. Run `python3 scripts/test_host.py`, package regression tests, and
   `python3 scripts/test_cmake.py`. Run `scripts/test_template.py` on a disposable
   PROS consumer using each supported kernel; never upload that consumer.
3. Review the final source, including new headers/tests and hardware evidence.
   Commit it and push a matching `guanid-vVERSION` tag. The tagged workflow must
   pass GCC/Clang ASan/UBSan, Windows MSVC CMake/CTest/install/consumer tests,
   packaging tests, and fresh PROS 4.2.1/4.2.2 consumer builds before publishing.
4. The release job downloads the host job's tested artifacts, verifies SHA256SUMS
   and the template manifest, then publishes the versioned template ZIP, complete
   buildable source ZIP, SHA256SUMS and a release-specific depot JSON. It uses
   the tagged source's changelog as release notes. Do not replace released assets.
5. Only after publication does it merge the new snapshot into `guanid-depot.json`
   on the `depot` branch. Existing version entries are retained; conflicting
   metadata for an existing version is rejected. Publication jobs are serialized.
   The stable URL is:
   `https://raw.githubusercontent.com/RandomPerson5571/guanid/depot/guanid-depot.json`.
6. Verify the public direct downloads and stable depot, then install the published
   ZIP in a fresh PROS project. If a run stops after publishing but before updating
   the depot, rerun the failed release job: it checks the published checksum list
   against the tested artifacts and only then retries the depot update.

The repository must be public for unauthenticated installation. GitHub Actions
must be allowed to write repository contents and create the `depot` branch;
branch policies must permit its normal fast-forward updates. Protect version
tags and review workflow changes as executable publishing permissions. The depot
branch is data only; generated commits do not change the library's source branch.

Reproducible packaging fixes ZIP file order, timestamps and permissions. The same
source and Python/zlib versions produce identical ZIP bytes. Generated `dist/`
files are ignored, and CI artifacts are retained independently of Git history.

Physical characterization reports and compile/test results are separate evidence.
See [drive-lab-results.md](drive-lab-results.md), [validation.md](validation.md)
and [hardware-validation.md](hardware-validation.md).
