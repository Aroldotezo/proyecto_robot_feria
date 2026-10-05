from collections.abc import Iterable

from robotic_arm.domain.classification_profile import ClassificationProfile
from robotic_arm.domain.exceptions.errors import ProfileNotFoundException
from robotic_arm.ports.profile_repository import ProfileRepository


class InMemoryProfileRepository(ProfileRepository):
    """Genera la persistencia de perfiles (se pierden al salir de la aplicacion)."""

    def __init__(self, profiles: Iterable[ClassificationProfile] = ()) -> None:
        self._profiles = {profile.name: profile for profile in profiles}

    def names(self) -> list[str]:
        return sorted(self._profiles)

    def load(self, name: str) -> ClassificationProfile:
        try:
            return self._profiles[name]
        except KeyError:
            raise ProfileNotFoundException(f"El perfil '{name}' no existe") from None

    def save(self, profile: ClassificationProfile) -> None:
        self._profiles[profile.name] = profile