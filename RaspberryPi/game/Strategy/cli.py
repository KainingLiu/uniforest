"""Shared launch options and hardware-free previews for competition entry points."""

from dataclasses import replace

from .transition_config import TransitionConfig
from .transition_switches import (
    TRANSITION_NAMES, TransitionSwitches, parse_selector,
    transition_enabled, validate_transition_selection,
)


def add_execution_arguments(parser):
    parser.add_argument('--transition-config', default=None,
                        help='JSON file of field-validated pickup/curve/alignment parameters')
    alignment = parser.add_mutually_exclusive_group()
    alignment.add_argument('--enable-fast-alignment', action='store_true',
                           help='enable fast pickup alignment using loaded calibration or trial parameters')
    alignment.add_argument('--disable-fast-alignment', action='store_true',
                           help='use original pickup alignment (default); other optimizations keep their choices')
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument('--enable-motion-planning', action='store_true',
                        help='smooth existing local move/turn sequences around original goals; retain original straight moves')
    motion.add_argument('--disable-motion-planning', '--classic-motion', dest='classic_motion', action='store_true',
                        help='use original route functions without motion-planning replacements (default)')
    moving_tag6 = parser.add_mutually_exclusive_group()
    moving_tag6.add_argument('--enable-moving-tag6', action='store_true',
                            help='opt into continuous Tag6 corrections during approach; requires --enable-motion-planning')
    moving_tag6.add_argument('--disable-moving-tag6', action='store_true',
                            help='keep original Tag recognition location and separate offset step (default); local smoothing stays enabled')
    parser.add_argument('--trial-optimizations', action='store_true',
                        help='load unvalidated pickup and alignment parameters; all seven optimizations require explicit enables')
    parser.add_argument('--trial-adaptive-blind', action='store_true',
                        help='with --trial-optimizations, preview neighbors to bound blind travel; unvalidated')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--enable-transitions', action='store_true',
                       help='enable all five cross-action transitions; pickup calibration is still required')
    group.add_argument('--disable-transitions', action='store_true',
                       help='disable all five cross-action transitions (default), then apply individual choices')
    names = ', '.join(TRANSITION_NAMES)
    for verb in ('enable', 'disable'):
        parser.add_argument(f'--{verb}-transition', action='append', default=[], metavar='TYPE[@PROFILE]',
                            help=f'{verb} a transition (default: disabled); repeatable; scoped choices take priority. Types: {names}')


def load_execution_config(args, plan):
    config = TransitionConfig.load(args.transition_config) if args.transition_config else TransitionConfig()
    if args.trial_adaptive_blind and (not args.trial_optimizations or not args.enable_fast_alignment):
        raise ValueError('adaptive trial requires --trial-optimizations and --enable-fast-alignment')
    if args.trial_optimizations:
        if args.transition_config:
            raise ValueError('trial settings cannot be mixed with a validated configuration file')
        from .optimizations.trial import trial_configuration
        config = trial_configuration(adaptive_blind=args.trial_adaptive_blind)
    overrides = {}
    for enabled, selectors in ((True, args.enable_transition), (False, args.disable_transition)):
        for selector in selectors:
            parse_selector(selector)
            if selector in overrides and overrides[selector] != enabled:
                raise ValueError(f'{selector}: cannot both enable and disable the same selector')
            overrides[selector] = enabled
    config = replace(config, switches=TransitionSwitches(
        disable_all=args.disable_transitions, overrides=overrides, enable_all=args.enable_transitions))
    if args.enable_fast_alignment and not config.alignments:
        raise ValueError('fast alignment requires alignment calibration or --trial-optimizations')
    if not args.enable_fast_alignment:
        config = replace(config, alignments={})
    config = replace(config, motion_planning_enabled=args.enable_motion_planning,
                     moving_tag6_enabled=args.enable_moving_tag6)
    validate_transition_selection(config, plan)
    return config


def print_plan(plan, config):
    from .settings import (FLAT_ROUTE_SPEED_MM_S,RAMP_ROUTE_SPEED_MM_S,
                           FLAT_ROUTE_ACCEL_MS,RAMP_ROUTE_ACCEL_MS)
    if config.trial_run:
        print('FIELD TRIAL: pickup/fast-alignment assumptions are unvalidated; route planning selected separately')
    print(f'{plan.name}: entry={plan.entry_anchor}, heading={plan.entry_heading_deg:g}')
    print(f'Route cruise targets: flat {FLAT_ROUTE_SPEED_MM_S:g} mm/s ({FLAT_ROUTE_ACCEL_MS} ms accel); '
          f'ramp {RAMP_ROUTE_SPEED_MM_S:g} mm/s ({RAMP_ROUTE_ACCEL_MS} ms accel); field verification pending')
    for index, step in enumerate(plan.steps, 1):
        print(f'{index:03d} {step.name}: {step.kind} [{step.profile}] {dict(step.parameters)}')
    if plan.needs_heading_zero:
        print('Entry requires a known --heading-zero-deg and physical placement at build approach.')
    print('Pickup transitions: ' + (', '.join(config.pickups) or 'disabled: no field calibration'))
    if config.curves:
        print('Legacy curve records (unused by local optimizer): ' + ', '.join(config.curves))
    if not config.motion_planning_enabled:
        print('Motion planning: classic (optimizers disabled)')
    else:
        print('Motion planning: local route smoothing; original straight moves and original route goals')
        from .optimizations.route_chain import grouped_steps
        for group in grouped_steps(plan.steps,enabled=True):
            if len(group)>1:
                print('Fused transport: ' + ' -> '.join(s.name for s in group)
                      + '; one transport to purple; settled straight ramp; no intermediate half-turn')
        print('No global-map start or Tag lock is required for ordinary motion; local blend limits await field verification.')
    print('Moving Tag6: ' + ('enabled: continuous visual correction and offset blending'
          if config.moving_tag6_enabled else 'disabled: original Tag alignment location and separate offset'))
    if config.motion_planning_enabled:
        from .optimizations.local_routes import LOCAL_ROUTES
        print('Route planning coverage:')
        seen_routes = set()
        for step in plan.steps:
            if step.kind != 'navigate':
                continue
            route = step.parameters['route']
            key = f'{step.profile}/{route}'
            if key in seen_routes:
                continue
            seen_routes.add(key)
            backend = ('local_route:existing_recipe' if route in LOCAL_ROUTES else
                       'classic:contact_or_visual_barrier')
            print(f'  {key}: {backend}')
        print('Single moves/turns and clearance retreats retain original controllers; contact/vision barriers remain explicit.')
        if config.moving_tag6_enabled:
            print('Tag6 joins only an adjacent existing Tag6/offset pair; unconfirmed approach keeps original alignment steps.')
    print('Fast pickup alignment: ' + (', '.join(config.alignments) or 'disabled: legacy alignment'))
    print('Adaptive blind preview: ' + (', '.join(key for key, policy in config.pickups.items()
        if policy.next_adaptive is not None and config.alignment(key.split('/')[0], 'orange') is not None)
        or 'disabled: configured fixed shift'))
    for key, policy in config.pickups.items():
        adaptive = policy.next_adaptive
        if adaptive is not None and config.alignment(key.split('/')[0], 'orange') is not None:
            print(f'  {key}: continuous={adaptive.continuous_pitch_mm:g}mm, '
                  f'observed_limit={adaptive.observed_distance_limit_mm:g}mm, '
                  f'unseen_search={adaptive.unseen_search_mm:g}mm, '
                  f'envelope={policy.next_blind.max_distance_mm:g}mm')
    print('Action transitions (enabled still requires runtime conditions):')
    seen = set()
    for name, source in validate_transition_selection(config, plan):
        selector = f'{name}@{source.profile}'
        if selector not in seen:
            enabled = transition_enabled(config, name, source)
            print(f'  {selector}: {"enabled" if enabled else "disabled"}')
            seen.add(selector)
    for selector in config.switches.overrides:
        if not any(item.split('@')[0] == selector for item in seen) and '@' not in selector:
            print(f'  {selector}: not present in this flow')
