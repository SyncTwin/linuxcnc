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
- [C] matiec 9c75953 и 13de41e собраны (make exit 0). SFC pulse.st + harness.c: @9c75953 28/28 PASS, @13de41e 10 FAIL (P/P1/P0 ×2). → SFC_RESULT.md
- [A] synctwin/mc-axis @950b590: configure --with-realtime=uspace --disable-gui (рецепт CI rip-headless; в synctwin/cloud-runtests @5f7590b рецепта сборки нет — там только тест home-delay-final-move), make exit 0, 0 warnings; halcompile --compile mc_axis.comp exit 0. runtests под root: FAIL (окружение: «Refusing to run as root»); под пользователем testrunner (как в CI): mc-axis PASS, mc-axis-vs-simple-tp PASS.
- INBOX_BUILD.md нет.
