# Utils/DatabaseManager.py

from __future__ import annotations

import os
import sqlite3
from dataclasses import dataclass
from typing import Dict, Any
from typing import List, Optional, Sequence, Tuple

@dataclass(frozen=True)
class RobotPosition:
    id: int
    name: str
    j_base: float
    j_bottom: float
    j_2: float
    j_3: float
    j_4: float
    j_gripper: float


class RobotPositionsDB:
    def __init__(self, db_path: str):
        self.db_path = db_path
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._ensure_db()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _ensure_db(self):
        with self._connect() as cur:
            cur.execute("""
                CREATE TABLE IF NOT EXISTS positions (
                    id          INTEGER PRIMARY KEY AUTOINCREMENT,
                    name        TEXT NOT NULL,
                    j_base      REAL NOT NULL,
                    j_bottom    REAL NOT NULL,
                    j_2         REAL NOT NULL,
                    j_3         REAL NOT NULL,
                    j_4         REAL NOT NULL,
                    j_gripper   REAL NOT NULL,
                    created_at  TEXT NOT NULL DEFAULT (datetime('now'))
                );
            """)

            # --- NEW: sequences table ---
            cur.execute("""
            CREATE TABLE IF NOT EXISTS sequences (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL UNIQUE,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP
            )
            """)

            # --- NEW: sequence blocks table ---
            cur.execute("""
            CREATE TABLE IF NOT EXISTS sequence_blocks (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                sequence_id INTEGER NOT NULL,
                ord INTEGER NOT NULL,
                block_type TEXT NOT NULL CHECK(block_type IN ('POSE','WAIT')),
                pose_id INTEGER,
                wait_seconds REAL,
                FOREIGN KEY(sequence_id) REFERENCES sequences(id) ON DELETE CASCADE
            )
            """)

            cur.execute("""
            CREATE INDEX IF NOT EXISTS idx_sequence_blocks_sequence_ord
            ON sequence_blocks(sequence_id, ord)
            """)

            # --- NEW: last_session key/value table ---
            cur.execute("""
            CREATE TABLE IF NOT EXISTS last_session (
                key TEXT PRIMARY KEY,
                value TEXT
            )
            """)

            cur.commit()

            # Ensure at least one default sequence exists
            if self.get_last_sequence_id() is None:
                seq_id = self.create_sequence_if_missing("Default")
                self.set_last_sequence_id(seq_id)

    # -----------------------------
    # Sequences API
    # -----------------------------
    def create_sequence_if_missing(self, name: str) -> int:
        name = name.strip() or "Unnamed"
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("SELECT id FROM sequences WHERE name = ?", (name,))
            row = cur.fetchone()
            if row:
                return int(row["id"])
            cur.execute("INSERT INTO sequences(name) VALUES(?)", (name,))
            conn.commit()
            return int(cur.lastrowid)

    def list_sequences(self) -> List[Tuple[int, str]]:
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("SELECT id, name FROM sequences ORDER BY name COLLATE NOCASE")
            return [(int(r["id"]), str(r["name"])) for r in cur.fetchall()]

    def get_sequence_name(self, sequence_id: int) -> Optional[str]:
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("SELECT name FROM sequences WHERE id = ?", (sequence_id,))
            row = cur.fetchone()
            return str(row["name"]) if row else None

    def delete_sequence(self, sequence_id: int):
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("DELETE FROM sequences WHERE id = ?", (sequence_id,))
            conn.commit()

    # -----------------------------
    # Sequence blocks persistence
    # -----------------------------
    def load_sequence_blocks(self, sequence_id: int) -> List[Dict[str, Any]]:
        """
        Returns blocks as dicts:
          - {"type":"POSE", "pose_id": int}
          - {"type":"WAIT", "seconds": float}
        """
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("""
                SELECT ord, block_type, pose_id, wait_seconds
                FROM sequence_blocks
                WHERE sequence_id = ?
                ORDER BY ord ASC
            """, (sequence_id,))
            out = []
            for r in cur.fetchall():
                if r["block_type"] == "POSE":
                    out.append({"type": "POSE", "pose_id": int(r["pose_id"])})
                else:
                    out.append({"type": "WAIT", "seconds": float(r["wait_seconds"] or 0.0)})
            return out

    def save_sequence_blocks(self, sequence_id: int, blocks: List[Dict[str, Any]]):
        """
        Replace-all save for MVP (simple + reliable).
        """
        with self._connect() as conn:
            cur = conn.cursor()

            cur.execute("DELETE FROM sequence_blocks WHERE sequence_id = ?", (sequence_id,))

            for i, b in enumerate(blocks):
                btype = b["type"]
                if btype == "POSE":
                    cur.execute("""
                        INSERT INTO sequence_blocks(sequence_id, ord, block_type, pose_id, wait_seconds)
                        VALUES(?, ?, 'POSE', ?, NULL)
                    """, (sequence_id, i, int(b["pose_id"])))
                elif btype == "WAIT":
                    cur.execute("""
                        INSERT INTO sequence_blocks(sequence_id, ord, block_type, pose_id, wait_seconds)
                        VALUES(?, ?, 'WAIT', NULL, ?)
                    """, (sequence_id, i, float(b.get("seconds", 0.0))))
                else:
                    raise ValueError(f"Unknown block type: {btype}")

            conn.commit()

    # -----------------------------
    # Last session
    # -----------------------------
    def set_last_sequence_id(self, sequence_id: int):
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("""
                INSERT INTO last_session(key, value)
                VALUES('last_sequence_id', ?)
                ON CONFLICT(key) DO UPDATE SET value=excluded.value
            """, (str(sequence_id),))
            conn.commit()

    def get_last_sequence_id(self) -> Optional[int]:
        with self._connect() as conn:
            cur = conn.cursor()
            cur.execute("SELECT value FROM last_session WHERE key='last_sequence_id'")
            row = cur.fetchone()
            if not row or row["value"] is None:
                return None
            try:
                return int(row["value"])
            except Exception:
                return None


    # ---------- CRUD ----------
    def add_position(
        self,
        name: str,
        joints: Sequence[float],
    ) -> int:
        """
        Adds a new stored position. Returns the inserted row id.
        joints must contain exactly 6 values in this order:
          [J_BASE, J_BOTTOM, J_2, J_3, J_4, J_GRIPPER]
        """
        if not name or not name.strip():
            raise ValueError("name must be a non-empty string")

        if len(joints) != 6:
            raise ValueError(f"Expected 6 joint values, got {len(joints)}")

        name = name.strip()
        j_base, j_bottom, j_2, j_3, j_4, j_gripper = map(float, joints)

        with self._connect() as conn:
            cur = conn.execute(
                """
                INSERT INTO positions (name, j_base, j_bottom, j_2, j_3, j_4, j_gripper)
                VALUES (?, ?, ?, ?, ?, ?, ?);
                """,
                (name, j_base, j_bottom, j_2, j_3, j_4, j_gripper),
            )
            conn.commit()
            return int(cur.lastrowid)

    def delete_position(self, position_id: int) -> bool:
        """
        Deletes a position by id. Returns True if a row was deleted.
        """
        with self._connect() as conn:
            cur = conn.execute("DELETE FROM positions WHERE id = ?;", (int(position_id),))
            conn.commit()
            return cur.rowcount > 0

    def get_position_by_id(self, position_id: int) -> Optional[RobotPosition]:
        with self._connect() as conn:
            row = conn.execute(
                """
                SELECT id, name, j_base, j_bottom, j_2, j_3, j_4, j_gripper
                FROM positions
                WHERE id = ?;
                """,
                (int(position_id),),
            ).fetchone()

        if row is None:
            return None

        return RobotPosition(
            id=int(row["id"]),
            name=str(row["name"]),
            j_base=float(row["j_base"]),
            j_bottom=float(row["j_bottom"]),
            j_2=float(row["j_2"]),
            j_3=float(row["j_3"]),
            j_4=float(row["j_4"]),
            j_gripper=float(row["j_gripper"]),
        )

    def search_positions_by_name(self, partial_name: str, limit: int = 50) -> List[RobotPosition]:
        """
        Fetch positions by partial name match (case-insensitive).
        Returns up to `limit` results ordered by newest first.
        """
        if partial_name is None:
            partial_name = ""
        partial_name = partial_name.strip()

        # Case-insensitive search: use LIKE with lower()
        pattern = f"%{partial_name.lower()}%"

        with self._connect() as conn:
            rows = conn.execute(
                """
                SELECT id, name, j_base, j_bottom, j_2, j_3, j_4, j_gripper
                FROM positions
                WHERE lower(name) LIKE ?
                ORDER BY id DESC
                LIMIT ?;
                """,
                (pattern, int(limit)),
            ).fetchall()

        return [
            RobotPosition(
                id=int(r["id"]),
                name=str(r["name"]),
                j_base=float(r["j_base"]),
                j_bottom=float(r["j_bottom"]),
                j_2=float(r["j_2"]),
                j_3=float(r["j_3"]),
                j_4=float(r["j_4"]),
                j_gripper=float(r["j_gripper"]),
            )
            for r in rows
        ]


# ---------------- Example usage ----------------
if __name__ == "__main__":
    db = RobotPositionsDB(db_path="data/robot_positions.sqlite")

    new_id = db.add_position(
        name="Home",
        joints=[0.0, 160.0, 60.0, 120.0, 90.0, 90.0],
    )
    print("Inserted id:", new_id)

    results = db.search_positions_by_name("ho")
    for r in results:
        print(r)

    ok = db.delete_position(new_id)
    print("Deleted:", ok)
