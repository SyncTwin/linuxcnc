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

## Шаг 2b — PP + ContinuousUpdate в mc_axis (готово)
- Коммиты: 2d363f8d (mc_axis), f978f0bd (tests).
- Решение (без правки cia402): continuous-update при drive-profile = 1 больше не отказ. Изменившиеся
  position/velocity/acceleration → новый set-point (pos-cmd, move-velocity, move-acceleration, 1 период
  move-execute); cia402.comp поднимает его с bit 5 (:658) → заменяет текущий.
- Рукопожатие: новый set-point только после того, как бит 12 0x6041 (берётся из уже существующего пина
  drv-statusword) поднялся и опустился после предыдущего — обход пробела cia402.comp :653–661.
  Пока рукопожатие не закончено, обновление «отложено» (sp_pending) и уходит, как только ack = 0.
- drv-move-done / armed: move_armed сбрасывается в 0 при КАЖДОМ новом set-point (как и при фронте);
  done засчитывается только после drv-move-done = 0 после последнего set-point → устаревшая 1 от старой
  цели не даёт Done. Если привод дошёл до старого set-point, а обновление ещё отложено, Done не
  выставляется: отложенный set-point уходит с места остановки (pp-done по :665 = ack 0, рукопожатие
  закончено). Без проводки drv-statusword обновления так и доходят — по одному после каждого pp-done.
- Ошибка relative + drive-profile исправлена попутно: cu_base брался ПОСЛЕ pos_cmd = tgt.
- Остаётся отказом (ошибка 7): jerk > 0 и deceleration ≠ acceleration при drive-profile = 1 — источник
  код cia402.comp: нет пина рывка, 0x6084 = 0x6083 (:649).
- Тест (tests/mc-axis, ось px): заглушка = виртуальный mc_axis drv в track mode (профиль привода, берёт
  новый set-point сразу), бит 12 = oneshot 50 мс от move-execute, drv-move-done = in-position И НЕ ack
  (как :665). 8 новых проверок.
- Результаты: с правкой `runtests tests/mc-axis` 1/1, 43 строки ok (было 35), 3 прогона подряд — все прошли.
- Обратный прогон (`/root/reverse.sh tests/mc-axis`: только mc_axis.comp ← f759789d, make, тест, возврат):
  исходный — 0/1, 31 ok, затем
  `FAIL: drive-profile ContinuousUpdate: accepted, no error 7`; с правкой — 1/1, 43 ok.

## Шаг 3 — jerk в track mode (готово)
- Факт по коду: `src/hal/components/simple_tp.comp` НЕ имеет входа jerk (пины target-pos, maxvel, maxaccel,
  enable). S-кривая с max_jerk есть только у планировщика jog в motion: `src/emc/motion/simple_tp.c`
  (simple_scurve_tp_update, :93–200) + `src/emc/tp/sp_scurve.c` (nextAccel/nextSpeed/stoppingDist).
  Эталон сравнения — он: tests/mc-axis-vs-simple-tp/test.sh собирает его в `ref_simple_tp` (как tests/blendmath),
  записанные входы mc_axis прогоняются через него период за периодом.
- Замеры опорного simple_tp.c (J=5000, A=200, V=50, dt=1 мс, прототип /root/proto, тот же код): проходит
  стоящую цель на ~3e-6 (100.000003156), jerk по ленте скорости до 27562 (> 5000), при снижении maxvel на ходу
  режет скорость за 1 период (a = 30000), в конце колеблется (v = -0.0003). Его концовку не копировал.
- Закон mc_axis (коммит 48b8a7d3): каждый период — рампа scurve_next к ±velocity (закон округления
  a² = 2·J·dv, тот же, что nextAccel() в simple_tp.c, с ограничением a), если после периода S-остановка
  ещё укладывается до цели; иначе — самая быстрая цель рампы w∈[0, velocity], при которой укладывается
  (regula falsi / Illinois, ≤ 30 шагов, конец при запасе ≤ 0.5·J·dt³). Длина остановки — та, что профиль
  реально проедет (симуляция по периодам, участок на пределе замедления — замкнутой формой). Движение от цели
  проходит v = 0 на полном замедлении (как simple_tp.c). Остаток < 0.5·J·dt³ из покоя — за один период.
  Абсолютные/относительные ходы с jerk по-прежнему отказ (ошибка 7).
- Отдельный коммит a9099698: «защёлка» последнего периода рампы (v = vreq, a = 0) давала скачок ускорения до
  1.5·J·dt (jerk до 1.5·J по ленте скорости). Теперь a = rem/dt, условие |rem| ≤ J·dt² и |rem/dt − a| ≤ J·dt.
  Прототип: 200000 случайных состояний — все сходятся, худшее 591 период, jerk ≤ J.
- Стоимость (прототип, Xeon 2.1 ГГц, не-RT контейнер; среднее, максимум искажён вытеснением):
  J=5000, A=200: ~3.3 мкс/период в среднем, до 3301 шагов симуляции за период;
  J=100, A=200 (A/J = 2 с): ~27 мкс/период, до 22432 шагов. Цена ∝ A/(J·dt).
- Тест tests/mc-axis-vs-simple-tp: cmpj.hal/cmpj.py (+ ref_simple_tp.c), 16 фаз: шаги, реверс, ближняя
  цель на ходу, enable 0 на ходу, движущаяся цель 20 ед/с, maxjerk 1000, шаги 0.001 / 0.00001 / 0.000017.
  Допуск TOL = maxvel·1.5·dt = 0.075: полпериода — схема интегрирования (mc_axis: новое a на весь период,
  позиция по средней скорости; simple_tp.c: среднее a и кубика), один период — начало торможения (simple_tp.c
  проверяет непрерывный stoppingDist, mc_axis — свою дискретную остановку). Не сравнивается снижение maxvel на
  ходу (опорный режет скорость за период).
  Свои проверки по ленте: |v| ≤ maxvel+1e-5, |a| ≤ maxaccel+1e-3, |j| ≤ maxjerk+2 (разрешение halsampler 6 знаков:
  0.5e-6 на значение → 2·0.5e-6/dt и 4·0.5e-6/dt²), нет перелёта стоящей цели (EPS 1e-5), покой точно на цели.
- Результат: 1/1, ok-строк 2; пик |v| 50.0000, |a| 200.000, |j| 5001.0; макс. |mc_axis − simple_tp.c| = 0.0314
  (фаза «движущаяся цель 20 ед/с»), 3 прогона подряд — все прошли.

## Шаг 4 — обратные прогоны (готово)
Команда: `/root/reverse.sh <тест>` — только src/hal/components/mc_axis.comp ← `git show f759789d:...`,
`make`, `runtests -n`, затем возврат рабочей копии и `make` (общее дерево не переписывается).
| тест | f759789d mc_axis.comp | с правкой |
|---|---|---|
| tests/mc-axis (43 проверки) | 0/1, 31 ok, `FAIL: drive-profile ContinuousUpdate: accepted (px error-id 7)` | 1/1, 43 ok |
| tests/mc-axis-vs-simple-tp | 0/1, 1 ok, `FAIL: mc_axis refused the jerk in track mode (error-id 7)` | 1/1, 2 ok |
| tests/mc-axis-vs-simple-tp, новый закон + старая защёлка | 0/1, `FAIL: jerk -7186.0 over maxjerk 5000.0 at period 19858, phase 16 (step +0.000017)` | 1/1 |
