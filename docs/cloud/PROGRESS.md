Ветка: cloud/mc-axis-v1

# PROGRESS — mc_axis: PP + ContinuousUpdate, jerk в track mode

## 2026-10-07 15:18 UTC — старт
- База: f759789d (+ f43ac890 с TASK.md). Работа последовательно, без подагентов.
- Шаг 1 (RIP-сборка uspace) — начат.

## Шаг 1 — RIP-сборка uspace (готово)
- Ubuntu 24.04, 4 CPU. apt: зависимости из debian/control.top.in (без docs/qt) + libedit-dev.
- `python3` в контейнере = 3.13 без tkinter → configure падает («TCL mismatch: 8.6 vs»).
  Обход: `PATH=/root/pybin:$PATH` (python3 → /usr/bin/python3.12) и
  `./configure --with-realtime=uspace --disable-gtk --disable-check-runtime-deps PYTHON=/usr/bin/python3.12`.
- `make -j4` — rc=0.
- Под root halrun отказывается («Refusing to run as root without fallback UID specified»);
  запуск: `useradd cnc; export RTAPI_UID=$(id -u cnc) RTAPI_FIFO_PATH=/root/.rtapi_fifo`.
- Исходный коммит f759789d (mc_axis.comp не менялся):
  - `runtests tests/mc-axis` — 1/1 тест прошёл; в result 35 строк `ok:`, 0 провалов.
  - `runtests tests/mc-axis-vs-simple-tp` — 1/1 тест прошёл; 1 строка `ok:`.

## Шаг 2a — анализ cia402 (две версии) для PP + ContinuousUpdate
Источники: `git show origin/synctwin/2.10:src/hal/components/cia402.comp` (695 строк) и upstream
`linuxcnc-ethercat/linuxcnc-ethercat` master 21bcc22d (06.10.2026) `src/cia402.c` (507 строк), клон только на чтение.

### synctwin/2.10 cia402.comp — что есть
- Пины PP: pp-execute (:151, фронт = новый set-point), pp-halt (:152, cw bit 8), pp-velocity (:153 → 0x6081),
  pp-accel (:154 → 0x6083 **и** 0x6084), drv-profile-velocity/accel/decel (:155–157), stat-setpoint-ack
  (:158, sw bit 12), pp-done (:159). param pp-mode (:183).
- Логика (:635–668): фронт pp-execute в ЛЮБОМ pp_state → pp_state = 1, pp-done = 0 (:653–656);
  pp_state 1: cw bit 5 + bit 4 (:658–660), переход в 2 при stat-setpoint-ack (:661);
  pp_state 2: bit 5 держится, bit 4 = 0, pp-done = !ack && target-reached (:663–665).
  0x6081/0x6083/0x6084 пишутся каждый период (:647–649), 0x6084 = 0x6083 всегда (:649).
- Ответ: перезапуск set-point на ходу cia402.comp умеет (повторный фронт pp-execute при bit 5 = 1 → новый
  фронт bit 4), НО только если предыдущий set-point уже квитирован (pp_state = 2). Пробелы:
  1. :653–661 — фронт pp-execute, пришедший пока pp_state == 1 (bit 4 ещё высок, ack не пришёл), не даёт
     нового фронта bit 4: новый 0x607A приводом не берётся, а pp-done (:665) потом сообщит о достижении
     СТАРОЙ цели. Нужна (не в этой задаче) защёлка «отложенный set-point»: опустить bit 4 после ack и поднять
     снова. mc_axis обходит это сам: новый set-point только после полного рукопожатия (ack 1, затем 0).
  2. :661 — в pp_state 1 устаревший ack = 1 от прошлого set-point сразу переводит в state 2 (bit 4 высок
     1 период). mc_axis не выдаёт новый set-point, пока ack не вернулся в 0, так что не попадает сюда.
  3. Нет пина 0x60A4 (profile jerk) и раздельной 0x6084 (:649) → jerk > 0 и deceleration ≠ acceleration при
     drive-profile = 1 остаются отказом (ошибка 7). Источник — код cia402.comp :649 и список пинов :151–159;
     что привод в PP вообще имеет объект рывка — НЕ ПОДТВЕРЖДЕНО ПЕРВОИСТОЧНИКОМ.
  4. Пин stat-setpoint-ack не нужен mc_axis отдельно: mc_axis уже читает сырой 0x6041 (pin drv-statusword),
     бит 12 берётся оттуда (как бит 13 для ошибки хоминга).
### upstream src/cia402.c — Profile Position НЕТ совсем
- opmode только 8/9/6 (:62–65, :495–505); controlword bit 4 только для хоминга (:505); bit 5 и bit 8 не
  формируются нигде; stat-target-reached только в CSP/CSV (:415–419); нет пинов pp-execute, pp-halt,
  pp-velocity, pp-accel, pp-done, stat-setpoint-ack, drv-profile-velocity/accel/decel, param pp-mode.
- Следствие: drive-profile = 1 mc_axis с upstream cia402.c не работает вообще (не только continuous-update);
  для него upstream нужен весь PP-блок synctwin (:148–159, :183, :207–208, :635–668) плюс п.1 выше.
### Различия пинов PP (synctwin cia402.comp vs upstream cia402.c)
| пин/строка | synctwin/2.10 | upstream |
|---|---|---|
| pp-execute (фронт → cw bit 4 + bit 5) | :151, :653–661 | нет |
| cw bit 5 change set immediately | :658, :664 (всегда с bit 4) | нет |
| cw bit 4 new set-point | :660 (PP), :684 (homing) | только homing :505 |
| pp-done | :159, :665 | нет |
| stat-setpoint-ack (sw bit 12) | :158, :373 | нет (bit 12 читается только как homed :423) |
| pp-velocity → 0x6081 | :153, :647 | нет |
| pp-accel → 0x6083/0x6084 | :154, :648–649 | нет |
| pp-halt → cw bit 8 | :152, :667 | нет |
| opmode 1 (PP) | :644, param pp-mode :183 | нет |
