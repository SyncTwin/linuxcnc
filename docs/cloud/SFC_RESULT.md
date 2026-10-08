# SFC_RESULT — matiec P/P1/P0/N/S/R, @9c75953 (после d6abc98) против @13de41e (до)

Источник: реальный прогон в этой сессии. Логи: `logs/C_sfc_9c75953.log`, `logs/C_sfc_13de41e.log`
(в логах есть потактовая таблица: состояние шагов и приращения счётчиков каждого действия на каждом скане).
Исходники: `sfc/pulse.st` (SFC), `sfc/harness.c` (C harness), `sfc/run.sh` (iec2c → gcc → прогон).

Сборка matiec: `autoreconf -i && ./configure && make` — exit 0 на обоих коммитах; `iec2c` (timeout 30) — exit 0, меток `{attribute}` нет.

Схема: `S0 --go--> (S1 || S3)`, `S1 --nxt--> S4 --nxt--> S2`, `(S2,S3) --back--> S0`.
S1: `A_P(P) A_P1(P1) A_P0(P0) A_N(N) A_S(S)`; S4: пусто; S2: `A_S(R) A_N3(R)`; S3: `A_N3(N)`.
Каждое тело действия увеличивает свой счётчик (всего) и отдельный счётчик «при S1.X/S4.X/S2.X/S3.X = TRUE».
Сценарии: S1 активен 1 скан (k=1) и 4 скана (k=4); S4 — 2 скана, S2 — 3 скана.

Ожидания: P/P1 — ровно 1 раз на фронт входа (в скане входа), P0 — ровно 1 раз на фронт выхода (в скане выхода);
N — k раз при активном шаге + 1 финальный скан (семантика форка e54dcd9: тело видит S1.X=FALSE);
S — работает в S1 и далее в S4 (защёлка), R в S2 гасит его с первого скана S2, остаётся 1 финальный скан.
«R против N другого шага» — только ФАКТ, без ожидания.

| квалификатор | сценарий | что считается | ожидание | факт @9c75953 | @9c75953 | факт @13de41e | @13de41e |
|---|---|---|---|---|---|---|---|
| P | S1 active 1 scan | total per activation | 1 | 1 | PASS | 2 | FAIL |
| P | S1 active 1 scan | in entry scan | 1 | 1 | PASS | 1 | PASS |
| P1 | S1 active 1 scan | total per activation (entry edge) | 1 | 1 | PASS | 2 | FAIL |
| P1 | S1 active 1 scan | in entry scan | 1 | 1 | PASS | 1 | PASS |
| P0 | S1 active 1 scan | total per deactivation (exit edge) | 1 | 1 | PASS | 2 | FAIL |
| P0 | S1 active 1 scan | in exit scan | 1 | 1 | PASS | 1 | PASS |
| P0 | S1 active 1 scan | in scan after exit | 0 | 0 | PASS | 1 | FAIL |
| P1 | S1 active 1 scan | in scan after exit | 0 | 0 | PASS | 0 | PASS |
| N | S1 active 1 scan | runs with S1.X=TRUE | 1 | 1 | PASS | 1 | PASS |
| N | S1 active 1 scan | total (k + final scan) | 2 | 2 | PASS | 2 | PASS |
| S | S1 active 1 scan | runs with S1.X=TRUE | 1 | 1 | PASS | 1 | PASS |
| S | S1 active 1 scan | runs with S4.X=TRUE (latched after S1) | 2 | 2 | PASS | 2 | PASS |
| S | S1 active 1 scan | runs with S2.X=TRUE (R active; final scan only) | 1 | 1 | PASS | 1 | PASS |
| S | S1 active 1 scan | total | 4 | 4 | PASS | 4 | PASS |
| R vs N (other step) | S1 active 1 scan | A_N3 runs while S2 (R) active | - | 3 | FACT | 3 | FACT |
| R vs N (other step) | S1 active 1 scan | A_N3 runs with S3.X=TRUE | - | 6 | FACT | 6 | FACT |
| R vs N (other step) | S1 active 1 scan | A_N3 total | - | 7 | FACT | 7 | FACT |
| P | S1 active 4 scans | total per activation | 1 | 1 | PASS | 2 | FAIL |
| P | S1 active 4 scans | in entry scan | 1 | 1 | PASS | 1 | PASS |
| P1 | S1 active 4 scans | total per activation (entry edge) | 1 | 1 | PASS | 2 | FAIL |
| P1 | S1 active 4 scans | in entry scan | 1 | 1 | PASS | 1 | PASS |
| P0 | S1 active 4 scans | total per deactivation (exit edge) | 1 | 1 | PASS | 2 | FAIL |
| P0 | S1 active 4 scans | in exit scan | 1 | 1 | PASS | 1 | PASS |
| P0 | S1 active 4 scans | in scan after exit | 0 | 0 | PASS | 1 | FAIL |
| P1 | S1 active 4 scans | in scan after exit | 0 | 0 | PASS | 0 | PASS |
| P | S1 active 4 scans | in 2nd active scan | 0 | 0 | PASS | 1 | FAIL |
| P1 | S1 active 4 scans | in 2nd active scan | 0 | 0 | PASS | 1 | FAIL |
| N | S1 active 4 scans | runs with S1.X=TRUE | 4 | 4 | PASS | 4 | PASS |
| N | S1 active 4 scans | total (k + final scan) | 5 | 5 | PASS | 5 | PASS |
| S | S1 active 4 scans | runs with S1.X=TRUE | 4 | 4 | PASS | 4 | PASS |
| S | S1 active 4 scans | runs with S4.X=TRUE (latched after S1) | 2 | 2 | PASS | 2 | PASS |
| S | S1 active 4 scans | runs with S2.X=TRUE (R active; final scan only) | 1 | 1 | PASS | 1 | PASS |
| S | S1 active 4 scans | total | 7 | 7 | PASS | 7 | PASS |
| R vs N (other step) | S1 active 4 scans | A_N3 runs while S2 (R) active | - | 3 | FACT | 3 | FACT |
| R vs N (other step) | S1 active 4 scans | A_N3 runs with S3.X=TRUE | - | 9 | FACT | 9 | FACT |
| R vs N (other step) | S1 active 4 scans | A_N3 total | - | 10 | FACT | 10 | FACT |

Итог: @9c75953 — 28/28 PASS (`FAILURES 0`, harness exit 0). @13de41e — 10 FAIL (`FAILURES 10`, exit 1).

## Выводы (по факту)
1. **До правки (13de41e)** каждое действие только с P/P1/P0 выполняется **2 раза** на фронт: второй раз — финальный
   скан (e54dcd9, `A = Q OR F_TRIG(Q)`) в следующем скане. Для k=1 у P/P1 второй запуск попадает в скан выхода
   (поэтому «in scan after exit» у P1 = 0), для k=4 — во второй активный скан; у P0 — в скан после выхода.
2. **После правки (9c75953)** P, P1, P0 — ровно 1 раз на фронт в обоих сценариях. N и S от правки не изменились
   (одинаковые счётчики до/после); diff сгенерированного C до/после — только 3 удалённые строки `A_P*_prev_Q = A_P*_Q`.
3. **S/R**: S защёлкивается (работает в S4, где нет ассоциаций), R в S2 гасит его с первого скана S2; в этом скане
   тело выполняется ещё 1 раз — финальный скан (S2.X=TRUE, Q=FALSE). Одинаково до и после.
4. **R НЕ гасит N другого шага (ФАКТ, оба коммита)**: `A_N3(R)` в S2 при `A_N3(N)` в параллельном S3 —
   A_N3 выполняется во всех 3 сканах активности S2. Причина в коде: R сбрасывает только `A_N3_stored`
   (S_FF), а `Q = Q_N | stored` — R не входит в Q (`stage4/generate_c/generate_c_sfc.cc:673` @9c75953, `/* Q = Q | stored */` в print_action_state_eval; генерация
   «Actions state evaluation»; в выходе `POUS.c`: `__SET_VAR(data__->,A_N3_Q,,__GET_VAR(data__->A_N3_Q) | __GET_VAR(data__->A_N3_stored))`).
   Совпадает ли это с ACTION_CONTROL IEC 61131-3 — вопрос оркестратору (не проверял текст стандарта в этой сессии).
5. Наблюдение: шаг не может стать активным и уйти в одном скане — переходы тестируются по снимку `Sx_X` на начале скана,
   поэтому минимальная активность шага — 1 скан действий, выход — в следующем скане.
