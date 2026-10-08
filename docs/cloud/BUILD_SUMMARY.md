# BUILD_SUMMARY (ветка claude/build-verify-linuxcnc-ca2wpp; всё ниже — по реальным прогонам этой сессии, Ubuntu 24.04, 4 CPU)

## A — synctwin/mc-axis @950b590 — ЗЕЛЁНОЕ
- Сборка: `./autogen.sh && ./configure --with-realtime=uspace --disable-gui --disable-manpages --disable-build-documentation && make -j4` → `make_exit=0`, 0 warnings (`logs/A_build.log`). Зависимости — список из `.github/scripts/install-deps-headless.sh`.
- В `synctwin/cloud-runtests` @5f7590b рецепта сборки нет (один коммит: тест `tests/home-delay-final-move`), взят рецепт CI job `rip-headless`.
- `halcompile --compile mc_axis.comp` → exit 0, `mc_axis.so` (`logs/A_halcompile.log`); в дереве `rtlib/mc_axis.so` собран штатно.
- `runtests tests/mc-axis` — **PASS**; `runtests tests/mc-axis-vs-simple-tp` — **PASS** («largest |mc_axis - simple_tp.c| position difference: 0.031385 … (TOL 0.075000)»). Логи `logs/A_mc-axis*.log`.
- Окружение: под root оба теста FAIL с «Refusing to run as root without fallback UID specified» (rtapi_app) — не дефект кода; зелёное получено под непривилегированным пользователем `testrunner`, как в CI.

## B — synctwin/2.10 @70609d28d — ЗЕЛЁНОЕ
- `git rev-parse origin/synctwin/2.10` = 70609d28dc457aeb… — совпадает.
- configure: `--with-realtime=uspace --disable-gtk` + `--disable-gui --disable-manpages --disable-build-documentation` (стоят только headless-зависимости) → `make -j4 V=1` exit 0 (`logs/B_make.log`). Warnings всей сборки: 2, оба `objects/hal/components/cia402.c:517` (unused parameter `prefix`, `extra_arg`).
- palletkins: `rtlib/palletkins.so` есть (экспорт kinematicsForward/Inverse/Type/TypeFlags/Switch/Switchable); warnings при -Wall — 0; при -Wall -Wextra — 0; на 70609d28~1 тем же вызовом -Wextra — 4 `unused parameter` (`palletkins.c:44`, `:60`) — коммит их снимает (`logs/B_palletkins.log`).
- iec2comp: `--help` exit 0; `blink.st` (`docs/cloud/iec2comp/blink.st`) с MATIEC=matiec@9c75953 → `.comp` + `.hal` exit 0; `halcompile --compile` exit 0 с 122 warnings (105 в рантайм-заголовках MATIEC, 17 `-Wunused-function` в самом .comp: неиспользуемые MC_* и `iec_issue`); `halcompile --install` + halrun (non-RT POSIX): `qx-0-0` FALSE→TRUE, `time` 0→481 — программа исполняется (`logs/B_iec2comp*.log`).
- Первый вход для iec2comp был ошибочным с моей стороны (located и обычная переменная в одном VAR) → iec2c «invalid located variable declaration»; сообщение iec2comp корректно указывает строку. Не дефект.

## C — SyncTwin/matiec synctwin/main @9c75953 — ЗЕЛЁНОЕ; @13de41e — КРАСНОЕ (ожидаемо, до d6abc98)
- `autoreconf -i && ./configure && make` → exit 0 на 9c75953 и 13de41e; iec2c под `timeout 30`, зависаний нет, `{attribute}` не использовался.
- `sfc/pulse.st` + `sfc/harness.c` (2 сценария: S1 активен 1 и 4 скана): @9c75953 **28/28 PASS**; @13de41e **10 FAIL** — P, P1, P0 выполняются по 2 раза на фронт (второй — финальный скан e54dcd9). N, S, R — одинаково до/после. Таблица — `SFC_RESULT.md`, логи `logs/C_sfc_*.log`.
- Diff сгенерированного C до/после — только 3 удалённые строки `A_P*_prev_Q = A_P*_Q` (заявленное «тот же C для графиков без P/P1/P0» на этом графике не проверялось — в нём есть P*).

## Найденное (ФАКТ) и вопросы
1. matiec: R в одном шаге НЕ гасит N того же действия в параллельном шаге — A_N3 выполнялся все 3 скана, пока S2 (`A_N3(R)`) активен. Адрес: `stage4/generate_c/generate_c_sfc.cc:673` (`Q = Q | stored`, R сбрасывает только `stored`). Вопрос: так ли по ACTION_CONTROL IEC 61131-3 (текст стандарта в сессии не проверял)? Патч не делал.
2. Финальный скан S-действия после R выполняется в скане, где шаг с R уже активен (тело видит S2.X=TRUE, Q=FALSE) — ФАКТ, до/после одинаково.
3. iec2comp: 17 `-Wunused-function` в сгенерированном .comp на любой программе (MC_* библиотека вставляется всегда) — косметика, вопрос: глушить ли `#pragma GCC diagnostic` в шаблоне.
4. Патчей в `docs/cloud/patches/` нет — чинить в рамках задания было нечего.
