# Бинарный протокол

Каждая UDP-датаграмма содержит ровно один пакет. Все многобайтовые поля записаны
little-endian. Структуры C++ не копируются напрямую; float и double переводятся
в целые битовые представления через `memcpy`, затем записываются по байтам.
Во время компиляции проверяются размеры и поддержка IEEE754.

| Смещение | Размер | Поле |
|---|---|---|
| 0 | 4 | magic = `0x44524F4E` |
| 4 | 2 | msg_id |
| 6 | 4 | seq |
| 10 | 2 | payload_len |
| 12 | payload_len | payload |
| 12 + payload_len | 2 | checksum |

Общая длина — `14 + payload_len`. Максимальный размер IPv4 UDP-датаграммы
с данными — 65507 байт. Checksum — сумма байт заголовка и payload по модулю 65536;
собственные два байта checksum в сумму не входят, что эквивалентно их обнулению.

Эталонный CMD_ARM с seq=1:

```text
4E 4F 52 44 0A 00 01 00 00 00 00 00 3E 01
```

Числовое значение magic при little-endian не образует буквальную строку DRON.
Это отмеченное расхождение в исходном ТЗ; реализация следует числу.

## Payload

| ID | Имя | Поля по порядку | Байты |
|---|---|---|---|
| 1 | HEARTBEAT | uint8 system_status | 1 |
| 2 | TELEMETRY | double lat, double lon, float alt, float yaw, float battery, uint8 armed, uint8 mode | 30 |
| 10 | CMD_ARM | нет | 0 |
| 11 | CMD_DISARM | нет | 0 |
| 12 | CMD_SET_MODE | uint8 mode | 1 |
| 13 | CMD_GOTO | double lat, double lon, float alt | 20 |
| 20 | ACK | uint16 cmd_msg_id, uint32 cmd_seq | 6 |
| 21 | NACK | uint16 cmd_msg_id, uint32 cmd_seq, uint8 reason | 7 |

`mode`: 0 MANUAL, 1 GUIDED, 2 RTL. `system_status`: 0 BOOT, 1 STANDBY,
2 ACTIVE, 3 CRITICAL. `armed`: 0 или 1.

`reason`: 1 NOT_ARMED, 2 BAD_MODE, 3 INVALID_ARGS, 4 BUSY, 5 OTHER.
Неподдерживаемый идентификатор входящей команды получает NACK OTHER.

Первый новый пакет каждого отправителя получает seq=1. Счетчик увеличивается
для всех типов сообщений; переполнение uint32 возвращает его к нулю.
Ретрай GCS повторяет прежний seq. Ответ на ретрай получает новый seq DroneSim,
но сохраняет исходные cmd_msg_id и cmd_seq внутри payload.

Заголовок проверяется до payload. Парсер не изменяет выходной объект при отказе.
Изменение длины с пересчитанным checksum тоже отвергается. В payload проверяются
не только размеры, но и конечность чисел, допустимые координаты и enum-значения.

Winsock использует отдельные правила для `sockaddr_in`: порт адреса записывается
через `htons`. Это не меняет little-endian прикладного протокола.
