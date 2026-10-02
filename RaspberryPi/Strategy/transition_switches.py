"""Per-transition launch choices; calibration and runtime guards remain binding."""

from dataclasses import dataclass, field
from types import MappingProxyType


TRANSITION_NAMES = (
    'next-cube', 'last-departure', 'purple-departure',
    'inspect-departure', 'build-return',
)
RETREATS = {'ground_delivery_reverse', 'orange_depart_reverse'}


def parse_selector(value):
    name, separator, profile = value.partition('@')
    if name not in TRANSITION_NAMES or separator and (not profile or '@' in profile):
        raise ValueError(f'unknown transition selector {value!r}; use TYPE or TYPE@PROFILE: '
                         + ', '.join(TRANSITION_NAMES))
    return name, profile if separator else None


@dataclass(frozen=True)
class TransitionSwitches:
    disable_all: bool = False
    overrides: object = field(default_factory=dict)
    enable_all: bool = False

    def __post_init__(self):
        if type(self.disable_all) is not bool:
            raise ValueError('disable_all must be boolean')
        if type(self.enable_all) is not bool:
            raise ValueError('enable_all must be boolean')
        if self.disable_all and self.enable_all:
            raise ValueError('cannot both enable and disable all transitions')
        choices = dict(self.overrides)
        for selector, enabled in choices.items():
            parse_selector(selector)
            if type(enabled) is not bool:
                raise ValueError('transition overrides must be boolean')
        object.__setattr__(self, 'overrides', MappingProxyType(choices))

    def enabled(self, name, profile):
        return self.overrides.get(f'{name}@{profile}',
                                 self.overrides.get(name, self.enable_all))


def transition_sites(steps):
    """Potential edges only; conditional pickups and live anchors still decide at runtime."""
    for index, (source, target) in enumerate(zip(steps, steps[1:])):
        method = source.parameters.get('method')
        same_profile = source.profile == target.profile
        if source.kind == 'grab_cube' and method in ('grap1', 'grap3'):
            if target.kind == 'acquire_cube' and same_profile:
                yield 'next-cube', source
            following = target
            for candidate in steps[index + 1:]:
                following = candidate
                optional = (candidate.kind in ('acquire_cube', 'grab_cube')
                            and candidate.parameters.get('conditional_on_purple')
                            and candidate.parameters.get('index') == 3)
                if not optional:
                    break
            if (following.kind == 'inspect_cargo' and following.profile == source.profile
                    and following.parameters.get('exit_route') in RETREATS):
                yield 'last-departure', source
        elif (source.kind == 'grab_cube' and method == 'grap2'
              and target.kind == 'navigate' and same_profile
              and target.parameters.get('route') == source.parameters.get('followup_route')):
            yield 'purple-departure', source
        elif (source.kind == 'inspect_cargo' and target.kind == 'navigate' and same_profile
              and source.parameters.get('exit_route') in RETREATS
              and target.parameters.get('route') in ('ground_to_delivery', 'orange_to_build')):
            yield 'inspect-departure', source
        elif (source.kind == 'build' and target.kind == 'navigate'
              and target.parameters.get('after_build') is True):
            yield 'build-return', source


def transition_enabled(config, name, source):
    return config.switches.enabled(name, source.profile)


def validate_transition_selection(config, plan):
    """Reject impossible explicit enables before creating Robot or executing a flow."""
    sites = tuple(transition_sites(plan.steps))
    present = {f'{name}@{source.profile}' for name, source in sites}
    for selector in config.switches.overrides:
        _, profile = parse_selector(selector)
        if profile is not None and selector not in present:
            raise ValueError(f'{selector}: no matching transition in {plan.name}')
    for name, source in sites:
        if name in ('inspect-departure', 'build-return') or not transition_enabled(config, name, source):
            continue
        policy = config.pickup(source.profile, source.parameters.get('method'))
        if policy is None or not (config.firmware_full_lift_validated or config.trial_run):
            raise ValueError(f'{name}@{source.profile}: pickup/full-lift calibration required')
        if name == 'next-cube':
            for profile in (policy.next_blind, policy.next_acquire):
                if profile is None or not (profile.validated or config.trial_run and profile.trial_enabled):
                    raise ValueError(f'{name}@{source.profile}: validated next_cube.blind/acquire required')
    return sites
