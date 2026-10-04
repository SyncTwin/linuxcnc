# SyncTwin/linuxcnc: коммиты, которых нет в апстриме

- **Дата проверки:** 2026-10-04
- **Ветка форка:** `synctwin/2.10` (`748ea7aa`)
- **База сравнения:** `upstream/master` (`056f2fd8`, LinuxCNC/linuxcnc)
- **Всего коммитов впереди:** 13 (`git log --oneline upstream/master..synctwin/2.10`)
- **Эквиваленты в апстриме (`git cherry`, строки с `-`):** 0

> **Важно:** в LinuxCNC/linuxcnc **нет ветки `2.10`** (есть `2.6`…`2.9` и `master`),
> поэтому `git fetch upstream 2.10` завершился ошибкой `couldn't find remote ref 2.10`.
> Сравнение сделано с `upstream/master`: `synctwin/2.10` ответвлена от него
> на merge-коммите `f7ccc4fc` (Merge PR #4611 hal64-halcmd-64-clean, 2026-10-02).

Две пары коммитов взаимно отменяют друг друга (`834ad7a0` ↔ `56fca84b`,
`3e7ffe04` ↔ `868c9275`), так что в итоговом дереве изменений в `homing.c` нет.
Фактически отличия от апстрима: компоненты cia402/trigwin, palletkins,
проверка M62–M68 и исправление delf/unloadrt в HAL с тестом.

| sha | тема | краткое описание | статус |
|-----|------|------------------|--------|
| `834ad7a0` | homing: wait HOME_DELAY before the final move | В `HOME_FINAL_MOVE_START` не хватало `break`, поэтому HOME_DELAY перед финальным перемещением к HOME не выдерживался. Отменён в `56fca84b`. | не отправлен (отменён в форке) |
| `7a495bcb` | hal: add cia402 component (CiA402 drive state machine) | Добавляет компонент `cia402.comp` — машину состояний привода CiA402 (из aicnc, Dominik Braun, GPL). SyncTwin-local: собирается в дереве вместо halcompile на целевой машине. | не отправлен |
| `900646a0` | hal: add trigwin component (ring buffer, window around a trigger) | Добавляет компонент `trigwin.comp` — кольцевой буфер, сохраняющий окно сигналов вокруг триггера. SyncTwin-local. | не отправлен |
| `597d8774` | kinematics: add palletkins (4R palletizer, flange kept level) | Новый модуль кинематики для 4R-паллетайзера с горизонтальным фланцем; геометрия задаётся HAL-пинами `palletkins.*`. SyncTwin-local. | не отправлен |
| `0f8af16c` | motion: refuse M62-M68 on a non-existent output | M62–M68 с P/E вне диапазона теперь отклоняются с ошибкой и останавливают программу, а не только пишут строку в лог. | не отправлен |
| `12f3be15` | hal: cia402 EXTRA_SETUP -> POST_EXPORT | В halcompile 2.10 `extra_setup()` вызывается до создания пинов; запись параметров переносится в `POST_EXPORT`, иначе модуль собирался без имён пинов и `loadrt` падал. | не отправлен |
| `3e7ffe04` | homing: no HOME_DELAY before the final move of a joint homed in place | Не ждать HOME_DELAY для оси, хоумящейся на месте, т.к. это ломало тесты с последовательным хоумингом. Отменён в `868c9275`. | не отправлен (отменён в форке) |
| `868c9275` | Revert "homing: no HOME_DELAY before the final move of a joint homed in place" | Откат `3e7ffe04`. | не отправлен (откат) |
| `56fca84b` | Revert "homing: wait HOME_DELAY before the final move" | Откат `834ad7a0`; поведение хоуминга возвращается к апстримному. | не отправлен (откат) |
| `df34e311` | tests: delf/unloadrt of a function in a running thread | Новый тест `tests/hal-delf-live`: циклы loadrt → addf → delf → unloadrt при запущенных потоках с компонентом, функция которого занимает поток. | не отправлен |
| `6fe0be7d` | hal: wait for the thread to leave a removed funct entry | `hal_del_funct_from_thread()`/`free_funct_struct()` больше не освобождают funct entry сразу, а ждут, пока RT-поток из неё выйдет; устраняет зависание или dlclose исполняемого кода. | не отправлен |
| `8a4e80b5` | tests: hal-delf-live component builds with the 2.10 halcompile | Правка `delfvictim.comp`, чтобы тестовый компонент собирался halcompile версии 2.10. | не отправлен |
| `748ea7aa` | hal: keep the links of a funct entry unlinked from a running thread | Удаление через `funct_entry_unlink()`: ссылки удалённой записи остаются указывать в список, а не на саму себя, иначе поток зацикливался на ней. | не отправлен |

## Как воспроизвести

```sh
git remote add upstream https://github.com/LinuxCNC/linuxcnc.git
git fetch upstream master
git log --oneline upstream/master..synctwin/2.10
git cherry -v upstream/master synctwin/2.10
```
