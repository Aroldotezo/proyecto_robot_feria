import sqlite3
from collections.abc import Iterator
from contextlib import contextmanager
from pathlib import Path

from robotic_arm.domain.arm_heights import ArmHeights
from robotic_arm.domain.arm_space import ArmPoint
from robotic_arm.domain.classification_profile import ClassificationProfile
from robotic_arm.domain.exceptions.errors import ProfileNotFoundException, ProfileStorageException
from robotic_arm.domain.geometry import NormalizedPoint, NormalizedRect
from robotic_arm.domain.reference_point import ReferencePoint
from robotic_arm.domain.zone import Zone, ZoneRole
from robotic_arm.ports.profile_repository import ProfileRepository

_SCHEMA_VERSION = 1

_SCHEMA = """
CREATE TABLE IF NOT EXISTS profiles (
    name   TEXT PRIMARY KEY,
    z_safe REAL NOT NULL,
    z_pick REAL NOT NULL,
    z_drop REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS zones (
    profile_name TEXT    NOT NULL REFERENCES profiles(name) ON DELETE CASCADE,
    position     INTEGER NOT NULL,
    id           TEXT    NOT NULL,
    label        TEXT    NOT NULL,
    role         TEXT    NOT NULL,
    x            REAL    NOT NULL,
    y            REAL    NOT NULL,
    width        REAL    NOT NULL,
    height       REAL    NOT NULL,
    category_id  TEXT,
    color        TEXT    NOT NULL,
    PRIMARY KEY (profile_name, position)
);
CREATE TABLE IF NOT EXISTS reference_points (
    profile_name TEXT    NOT NULL REFERENCES profiles(name) ON DELETE CASCADE,
    position     INTEGER NOT NULL,
    image_x      REAL    NOT NULL,
    image_y      REAL    NOT NULL,
    arm_x        REAL    NOT NULL,
    arm_y        REAL    NOT NULL,
    PRIMARY KEY (profile_name, position)
);
"""


class SqliteProfileRepository(ProfileRepository):
    """Stores classification profiles in a local SQLite file (stdlib only, same on Windows and Linux)."""

    def __init__(self, path: Path) -> None:
        self._path = path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            raise ProfileStorageException(f"No se pudo crear la carpeta de datos ({path.parent}): {e}") from e
        with self._connection() as connection:
            self._prepare_schema(connection)

    def names(self) -> list[str]:
        with self._connection() as connection:
            return [row[0] for row in connection.execute("SELECT name FROM profiles ORDER BY name")]

    def load(self, name: str) -> ClassificationProfile:
        with self._connection() as connection:
            heights = connection.execute(
                "SELECT z_safe, z_pick, z_drop FROM profiles WHERE name = ?", (name,)
            ).fetchone()
            if heights is None:
                raise ProfileNotFoundException(f"El perfil '{name}' no existe")
            zones = connection.execute(
                "SELECT id, label, role, x, y, width, height, category_id, color "
                "FROM zones WHERE profile_name = ? ORDER BY position",
                (name,),
            ).fetchall()
            points = connection.execute(
                "SELECT image_x, image_y, arm_x, arm_y FROM reference_points WHERE profile_name = ? ORDER BY position",
                (name,),
            ).fetchall()

        try:
            return ClassificationProfile(
                name=name,
                zones=tuple(
                    Zone(zone_id, label, ZoneRole(role), NormalizedRect(x, y, width, height), category_id, color)
                    for zone_id, label, role, x, y, width, height, category_id, color in zones
                ),
                reference_points=tuple(
                    ReferencePoint(NormalizedPoint(image_x, image_y), ArmPoint(arm_x, arm_y))
                    for image_x, image_y, arm_x, arm_y in points
                ),
                heights=ArmHeights(*heights),
            )
        except ValueError as e:
            raise ProfileStorageException(f"El perfil '{name}' guardado está dañado: {e}") from e

    def save(self, profile: ClassificationProfile) -> None:
        heights = profile.heights
        with self._connection() as connection: 
            connection.execute(
                "INSERT INTO profiles (name, z_safe, z_pick, z_drop) VALUES (?, ?, ?, ?) "
                "ON CONFLICT(name) DO UPDATE SET "
                "z_safe = excluded.z_safe, z_pick = excluded.z_pick, z_drop = excluded.z_drop",
                (profile.name, heights.z_safe, heights.z_pick, heights.z_drop),
            )
            connection.execute("DELETE FROM zones WHERE profile_name = ?", (profile.name,))
            connection.execute("DELETE FROM reference_points WHERE profile_name = ?", (profile.name,))
            connection.executemany(
                "INSERT INTO zones (profile_name, position, id, label, role, x, y, width, height, category_id, color) "
                "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
                [
                    (
                        profile.name, position, zone.id, zone.label, zone.role.value,
                        zone.region.x, zone.region.y, zone.region.width, zone.region.height,
                        zone.category_id, zone.color,
                    )
                    for position, zone in enumerate(profile.zones)
                ],
            )
            connection.executemany(
                "INSERT INTO reference_points (profile_name, position, image_x, image_y, arm_x, arm_y) "
                "VALUES (?, ?, ?, ?, ?, ?)",
                [
                    (profile.name, position, point.image.x, point.image.y, point.arm.x, point.arm.y)
                    for position, point in enumerate(profile.reference_points)
                ],
            )

    # ------------------------------------------------------------------ internals

    @contextmanager
    def _connection(self) -> Iterator[sqlite3.Connection]:
        """Short-lived connection per operation (no thread or file-lock surprises). Commits on success."""
        try:
            connection = sqlite3.connect(self._path)
            try:
                connection.execute("PRAGMA foreign_keys = ON")
                with connection:  # transaction: commit, or rollback if the body raises
                    yield connection
            finally:
                connection.close()
        except sqlite3.Error as e:
            raise ProfileStorageException(f"Error al acceder a la base de datos de perfiles ({self._path}): {e}") from e

    @staticmethod
    def _prepare_schema(connection: sqlite3.Connection) -> None:
        version = connection.execute("PRAGMA user_version").fetchone()[0]
        if version > _SCHEMA_VERSION:
            raise ProfileStorageException(
                "La base de datos de perfiles fue creada por una versión más nueva de la aplicación"
            )
        if version == 0:
            connection.executescript(_SCHEMA)
            connection.execute(f"PRAGMA user_version = {_SCHEMA_VERSION}")