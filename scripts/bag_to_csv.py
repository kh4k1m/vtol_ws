#!/usr/bin/env python3
"""
Конвертация rosbag2 (каталог с .db3 + metadata) в CSV по топикам.

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 scripts/bag_to_csv.py logs/sim_bag_20250422-120000

Или путь к одному .db3 — возьмётся родительский каталог с metadata.

Сообщения с тяжёлыми полями (картинка, байты) — поле data/часть массивов можно обрезать
флагом --max-array (по умолчанию 9 для коротких векторов, для Image — только shape + len data).
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Set, Tuple

# ROS: нужен source install
try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print("Import error: {}. Сначала: source /opt/ros/humble/setup.bash && source install/setup.bash".format(e), file=sys.stderr)
    sys.exit(1)


def resolve_bag_uri(path: str) -> str:
    p = os.path.abspath(path)
    if os.path.isfile(p) and p.endswith(".db3"):
        return os.path.dirname(p)
    if os.path.isdir(p):
        return p
    raise SystemExit("Ожидается каталог rosbag2 или путь к .db3 файлу: {!r}".format(path))


def require_bag_metadata(bag_uri: str) -> None:
    meta = os.path.join(bag_uri, "metadata.yaml")
    if not os.path.isfile(meta):
        raise SystemExit(
            "В {!r} нет metadata.yaml. Полный rosbag2 — это каталог с .db3 и metadata.yaml; "
            "без файла метаданных rosbag2_sqlite3 часто падает с «SQLite error (10): disk I/O error». "
            "Скопируйте metadata с машины записи или восстановите его (путь к .db3 в relative_file_paths)."
            .format(bag_uri)
        )


def flatten_dict(
    d: Any,
    parent: str = "",
    sep: str = ".",
    max_list_items: int = 64,
) -> OrderedDict:
    """
    Словарь/список/скаляры → плоский OrderedDict.
    Массивы обрезаются по длине max_list_items (дальше + '_truncated').
    """
    out: "OrderedDict[str, Any]" = OrderedDict()
    if isinstance(d, dict):
        for k, v in d.items():
            pk = f"{parent}{sep}{k}" if parent else str(k)
            out.update(flatten_value(pk, v, sep, max_list_items))
    else:
        out.update(flatten_value(parent, d, sep, max_list_items))
    return out


def flatten_value(
    name: str,
    v: Any,
    sep: str,
    max_list_items: int,
) -> "OrderedDict[str, Any]":
    out: "OrderedDict[str, Any]" = OrderedDict()
    if v is None or isinstance(v, (bool, int, float, str)):
        out[name] = v
    elif isinstance(v, dict):
        for k, val in v.items():
            sub = f"{name}{sep}{k}" if name else str(k)
            out.update(flatten_value(sub, val, sep, max_list_items))
    elif isinstance(v, (list, tuple)):
        n = min(len(v), max_list_items)
        for i in range(n):
            out.update(
                flatten_value(
                    f"{name}{sep}{i}" if name else str(i), v[i], sep, max_list_items
                )
            )
        if len(v) > max_list_items:
            out[name + sep + "truncated_len"] = len(v) - n
    else:
        out[name] = repr(v)
    return out


def read_all_messages(
    bag_uri: str,
) -> Tuple[Dict[str, str], List[Tuple[str, int, bytes]]]:
    storage_options = rosbag2_py.StorageOptions(uri=bag_uri, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map: Dict[str, str] = {}
    for t in reader.get_all_topics_and_types():
        type_map[t.name] = t.type

    messages: List[Tuple[str, int, bytes]] = []
    while reader.has_next():
        s = reader.read_next()
        # Humble: обычно (topic, serialized_data, t_ns), но переставим байты/время по типам
        if isinstance(s, tuple) and len(s) >= 3:
            a, b, c = s[0], s[1], s[2]
            if isinstance(a, str) and isinstance(b, bytes) and isinstance(c, (int, float)):
                topic, data, t = a, b, int(c)
            elif isinstance(a, str) and isinstance(c, bytes) and isinstance(b, (int, float)):
                topic, data, t = a, c, int(b)
            else:
                topic, data, t = a, b, int(c)  # надежда на (topic, data, t)
            messages.append((topic, t, data))
        elif hasattr(s, "topic_name"):
            ts = getattr(s, "time_stamp", None) or getattr(s, "timestamp", 0)
            messages.append((s.topic_name, int(ts), s.serialized_data))
        else:
            print("Неизвестный формат read_next():", type(s), file=sys.stderr)
    return type_map, messages


def main() -> None:
    ap = argparse.ArgumentParser(description="rosbag2 → CSV (по одному файлу на топик).")
    ap.add_argument("bag", help="каталог bag или путь к .db3")
    ap.add_argument(
        "-o",
        "--output-dir",
        default=None,
        help="каталог для CSV (по умолчанию: <bag_basename>_csv рядом с bag).",
    )
    ap.add_argument(
        "--max-array",
        type=int,
        default=24,
        help="макс. элементов вложенного массива при раскрытии (остальное — _truncated_len).",
    )
    args = ap.parse_args()

    bag_uri = resolve_bag_uri(args.bag)
    require_bag_metadata(bag_uri)
    out_dir = args.output_dir or (bag_uri + "_csv")
    os.makedirs(out_dir, exist_ok=True)
    print("Bag:", bag_uri, "->", out_dir)

    type_map, raw_msgs = read_all_messages(bag_uri)
    if not raw_msgs:
        print("В bag нет сообщений.", file=sys.stderr)
        sys.exit(1)

    by_topic: Dict[str, List[Tuple[int, bytes]]] = {}
    for topic, t_ns, data in raw_msgs:
        by_topic.setdefault(topic, []).append((t_ns, data))

    for topic, rows in by_topic.items():
        tname = type_map.get(topic, "")
        if not tname:
            print("Пропуск (нет типа):", topic, file=sys.stderr)
            continue
        try:
            msg_cls = get_message(tname)
        except Exception as e:
            print("Тип {}: {}".format(tname, e), file=sys.stderr)
            continue

        all_rows: List[Dict[str, Any]] = []
        for t_ns, data in rows:
            try:
                msg = deserialize_message(data, msg_cls)
            except Exception as e:
                print("Ошибка deserialize", topic, e, file=sys.stderr)
                continue
            od = message_to_ordereddict(msg)
            if tname == "sensor_msgs/msg/Image" and "data" in od:
                d = od.get("data")
                if hasattr(d, "__len__") and d is not None:
                    od["data_bytes"] = len(d)
                od.pop("data", None)
            flat = flatten_dict(od, parent="", max_list_items=args.max_array)
            rec = OrderedDict()
            rec["time_stamp_ns"] = t_ns
            rec["time_sec"] = t_ns * 1e-9
            for k, v in flat.items():
                if v is not None and not isinstance(v, (bool, int, float, str)):
                    rec[k] = repr(v) if not isinstance(v, (list, tuple, dict)) else str(v)
                else:
                    rec[k] = v
            all_rows.append(rec)

        if not all_rows:
            continue
        colset: "Set[str]" = set()
        for r in all_rows:
            colset.update(r.keys())
        head = [x for x in ("time_stamp_ns", "time_sec") if x in colset]
        rest = sorted(c for c in colset if c not in head)
        fieldnames = head + rest

        safe_topic = topic.replace(os.sep, "_").lstrip("_").replace("/", "_") or "topic"
        out_path = os.path.join(out_dir, safe_topic + ".csv")

        with open(out_path, "w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(
                f, fieldnames=fieldnames, extrasaction="ignore", restval=""
            )
            w.writeheader()
            for r in all_rows:
                w.writerow({k: r.get(k, "") for k in fieldnames})
        print(" ", topic, "->", out_path, "rows=", len(all_rows))

    print("Готово.")


if __name__ == "__main__":
    main()
