# robot_positions_db.py
# Minimal SQLite helper for storing "teach & repeat" joint positions.

from __future__ import annotations

import os
import sqlite3
from dataclasses import dataclass
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
        self._ensure_db()

    # ---------- Core ----------
    def _connect(self) -> sqlite3.Connection:
        # Ensure directory exists
        folder = os.path.dirname(os.path.abspath(self.db_path))
        if folder and not os.path.exists(folder):
            os.makedirs(folder, exist_ok=True)

        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def _ensure_db(self) -> None:
        """
        Creates the database file and required table if they do not exist.
        Safe to call multiple times.
        """
        with self._connect() as conn:
            conn.execute("""
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
            conn.execute("""
                CREATE INDEX IF NOT EXISTS idx_positions_name
                ON positions(name);
            """)
            conn.commit()

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
