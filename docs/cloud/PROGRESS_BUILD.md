Ветка результата: claude/build-verify-linuxcnc-ca2wpp (среда сессии разрешает push только в неё; cloud/build-verify-v1 — не моя целевая)

# PROGRESS_BUILD

Задание: docs/cloud/BUILD_TASK.md

## План
1. Часть C (matiec): clone, checkout 9c75953, сборка iec2c; SFC pulse.st + C harness; прогон @9c75953 и @13de41e.
2. Часть A (synctwin/mc-axis): apt-зависимости по рецепту synctwin/cloud-runtests, сборка uspace, runtests tests/mc-axis, tests/mc-axis-vs-simple-tp.
3. Часть B (synctwin/2.10 @70609d28d): autogen/configure/make, palletkins warnings, iec2comp.py.
4. BUILD_SUMMARY.md.

## Журнал
- [старт] PROGRESS + план.
