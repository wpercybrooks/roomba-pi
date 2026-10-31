# Third-party attribution

This project is MIT licensed (see `LICENSE`). It borrows the following code.

| Component | Version | Licence | Author | What we changed | Where |
| --- | --- | --- | --- | --- | --- |
| [pycreate2](https://github.com/MomsFriendlyRobotCompany/pycreate2) | 0.8.0 (released 2020-04-29) | MIT | Kevin Walchko; portions copyright Damon Kohler, Jonathan Le Roux, Brandon Pomeroy (full notice in `vendor/pycreate2/LICENSE`) | Vendored unmodified apart from seven documented patches (inert `__del__`, gated `stop()`, `passive()`, read-until-80 `get_sensors()`, exclusive-lock/silent open, `sane`/`raw` fields, version fallback). See `vendor/pycreate2/PATCHES.md`. | `vendor/pycreate2/` |
| [pyserial](https://github.com/pyserial/pyserial) | >=3.5 | BSD-3-Clause | Chris Liechti and contributors | Not vendored, not modified — the only runtime dependency. | `pyproject.toml` |

The pycreate2 MIT notice travels inside the package directory (`vendor/pycreate2/LICENSE`) as the licence
requires. Upstream provenance is recorded in `vendor/pycreate2/UPSTREAM`.
