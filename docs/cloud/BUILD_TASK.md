# Облачное задание: сборка и прогон наших веток LinuxCNC + matiec (ветка результата `cloud/build-verify-v1`)

Ты — отдельная облачная сессия без подагентов. Репо `SyncTwin/linuxcnc` (публичный форк). Журнал шагов — `docs/cloud/PROGRESS_BUILD.md`.

## Границы (§0)
- Ветка результата: `cloud/build-verify-v1` (этот репо). Первый push ≤15 мин (PROGRESS + план), далее push после КАЖДОГО шага. В `master`/`synctwin/*` не пушить, PR не открывать. Не пускает в названную ветку — пушить в разрешённую и написать её имя первой строкой PROGRESS_BUILD.md.
- После каждого шага: `git pull --rebase` и прочитай `docs/cloud/INBOX_BUILD.md` (если есть) — указания оркестратора важнее этого задания.
- Только чтение чужого: ветки `synctwin/*` и matiec НЕ правятся; исправления — только как патч-файл в `docs/cloud/patches/` с пояснением. Секретов нет и не нужны.
- Числа без источника — `ПРЕДПОЛОЖЕНИЕ, не замер`. Зелёное — только по реальному прогону; красное — с логом.

## Цель
Дать зелёное/красное по трём сборкам и одной проверке SFC. Три части независимы — продолжай со следующей, если одна красная.

## Часть A — `synctwin/mc-axis`
Вход: `git fetch origin synctwin/mc-axis` (950b590); компонент `src/hal/components/mc_axis.comp`; тесты `tests/mc-axis`, `tests/mc-axis-vs-simple-tp`; рецепт сборки/прогона runtests в ветке `synctwin/cloud-runtests` (5f7590b) — прочитай, как она собирает в контейнере.
Сделать: поставить зависимости (apt), собрать минимум для halcompile/rtapi sim; `halcompile` на `mc_axis.comp` (установка/сборка .so для sim); `runtests tests/mc-axis` и `runtests tests/mc-axis-vs-simple-tp`.
Готово: по каждому тесту `PASS/FAIL` + хвост лога в `docs/cloud/logs/A_*.log`.

## Часть B — `synctwin/2.10` @ 70609d28d
Вход: `git fetch origin synctwin/2.10`; убедись `git rev-parse` = 70609d28d (иначе запиши расхождение). Есть `src/hal/utils/iec2comp.py` и `src/emc/kinematics/palletkins.c` (патчи 0004/0005).
Сделать: `./autogen.sh && ./configure --with-realtime=uspace --disable-gtk` (по необходимости — опции из cloud-runtests), `make -j`; собралась ли `palletkins` (kinematics module, проверь .so и warnings в её компиляции); `iec2comp.py --help` / запуск на тривиальном входе, если для него нужен iec2c — после части C.
Готово: make exit code, список warnings из `palletkins.c`/`iec2comp`, `docs/cloud/logs/B_make.log` (хвост ≤300 строк).

## Часть C — `SyncTwin/matiec` `synctwin/main` @ 9c75953
Вход: `git clone https://github.com/SyncTwin/matiec` (публичный), `git checkout 9c75953` (ветка synctwin/main). Сборка: autoreconf -i && ./configure && make (iec2c, iec2iec). В контейнере `iec2c` на `{attribute}` может зависать — все запуски `timeout 30`, меток `{attribute}` не использовать.
Сделать: написать SFC-программу (`docs/cloud/sfc/pulse.st`) с шагом и переходами, у которого действия квалификаторов P, P1, P0, N, S, R; скомпилировать iec2c → C, добавить harness на C (gcc, считает числа вызовов тела действия на каждый фронт активации/деактивации шага). Проверить:
  1. P1 — ровно 1 раз на фронт входа (не 2), P0 — ровно 1 раз на фронт выхода, P — ровно 1 раз на фронт (шаг активен 1 такт и несколько тактов — разные сценарии);
  2. N — выполняется каждый такт, пока шаг активен; S/R — установка и сброс (R гасит S и N других шагов? запиши ФАКТ, не ожидание);
  3. сравнить с поведением до правки: `git checkout 13de41e` (до d6abc98) и тот же harness — таблица «до/после».
Готово: `docs/cloud/SFC_RESULT.md` — таблица «квалификатор · сценарий · счётчик ожидание · счётчик факт · PASS/FAIL» для @9c75953 и @13de41e; исходники harness и логи в ветке.

## Критерий «готово» всей сессии
`docs/cloud/BUILD_SUMMARY.md` (≤40 строк): A/B/C — зелёное/красное по каждому тесту/сборке, главные строки логов, найденные дефекты с адресом (файл:строка), вопросы. Не писать «сделано», если не прогнано.

## Нельзя
Править чужие ветки · PR · подагенты · секреты · числа на глаз · глушить красное.
