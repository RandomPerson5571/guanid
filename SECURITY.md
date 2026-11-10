# Security and robot safety reports

Do not use guanid as a safety controller. Motor communication failures, task
termination, power loss, blocked callbacks, and gravity loads can prevent a
software stop from bringing a mechanism to a safe state. Use independent robot
disable controls and follow the [physical validation protocol](docs/hardware-validation.md).
For a stop or disable report, include the control mode, trigger, last observed
voltage and any measured delay.

For ordinary defects, report the version, kernel/toolchain, configuration, minimal
reproduction and expected/actual result through the hosting repository's issue
tracker. Remove credentials, private paths and team-sensitive captures first.
For a suspected vulnerability, use the repository's private vulnerability report
feature when available. If that feature is unavailable, contact the repository
owner privately rather than disclosing an exploitable issue in a public log.
No dedicated security response service or response-time guarantee is offered.

Only the latest released version receives fixes. Older captures and manually
modified headers may need a reproducer against that version. Desktop CSV input
is intended for your own robot logs; review external files before analyzing them.
