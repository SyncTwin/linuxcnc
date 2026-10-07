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
