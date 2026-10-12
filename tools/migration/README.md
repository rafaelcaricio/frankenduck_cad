# Original port utilities

These utilities read retained local reference inputs. They are
outside every maintained check suite.

- `prepare_port.py`: recreate the original source from the pinned snapshot.
  It overwrites local CAD edits and requires the explicit
  `--replace-local-source` flag. Do not use it for ordinary revisions.
- `geometry_audit.py`: original reconstruction/baseline comparison. Current
  edited parts intentionally differ from those baselines.
- `face_equivalence.py`: original face-plate comparison against the imported
  reference STEP.
- `update_repaired_audit.py`: retry failed rows in the historical audit.

Run from the project root with `.venv/bin/python tools/migration/<name>.py`
and `PYTHONPATH=src`. Historical results live in `reports/archive/import/`;
the original r91 evidence also remains under `revisions/r91_import/`.
