"""Per-run PlanD cargo evidence and independently committed building sites."""

from dataclasses import dataclass
import time
from typing import Optional


SOURCES = {'task1-1': (3, 'A'), 'task1-2': (4, 'B')}


def _validate_count(count):
    if count is not None and (type(count) is not int or count not in range(4)):
        raise ValueError('PlanD cargo count must be None or an integer from 0 to 3')


@dataclass(frozen=True)
class CargoResult:
    round_id: int
    site_id: str
    count: Optional[int]
    refill_used: bool = False
    source: str = 'vision'

    def __post_init__(self):
        _validate_count(self.count)
        if (self.round_id, self.site_id) not in SOURCES.values():
            raise ValueError('PlanD cargo round/site identity mismatch')


@dataclass
class SiteRecord:
    source_round: int
    cargo: Optional[CargoResult] = None
    observed_cargo_count: Optional[int] = None
    deposited_count: Optional[int] = None
    height_status: str = 'pending'
    unload_completed_at: Optional[float] = None
    topping_status: str = 'pending'
    unknown_recheck_used: bool = False
    # Count is inferred from cargo and successful unloading; no stack camera
    # confirms that all deposited cubes remained upright at the destination.
    height_evidence: str = 'unverified'


class PlanDState:
    """Created once by the runner; never reused after cancellation or completion."""

    def __init__(self):
        self.sites = {'A': SiteRecord(3), 'B': SiteRecord(4)}
        self.refill_source_task = None
        self.closed = False

    def _check_open(self):
        if self.closed:
            raise RuntimeError('PlanD run is closed; start a fresh run')

    def pending_site(self, source_task):
        self._check_open()
        _, site_id = SOURCES[source_task]
        site = self.sites[site_id]
        if site.unload_completed_at is not None:
            raise RuntimeError(f'PlanD site {site_id} was already unloaded')
        return site

    def record_cargo(self, source_task, count, *, refill_used=False):
        site = self.pending_site(source_task)
        round_id, site_id = SOURCES[source_task]
        cargo = CargoResult(round_id, site_id, count, refill_used)
        site.cargo = cargo
        site.observed_cargo_count = count
        return cargo

    def begin_refill(self, source_task):
        site = self.pending_site(source_task)
        if self.refill_source_task is not None or site.cargo is None or site.cargo.refill_used:
            raise RuntimeError('PlanD permits one refill for each original collection round')
        self.refill_source_task = source_task

    def finish_refill(self, source_task):
        self._check_open()
        if self.refill_source_task != source_task:
            raise RuntimeError('PlanD refill source mismatch')
        self.refill_source_task = None

    def require_refill_result(self, source_task):
        site = self.pending_site(source_task)
        if (self.refill_source_task != source_task or site.cargo is None
                or not site.cargo.refill_used):
            raise RuntimeError('PlanD refill omitted its final cargo observation')
        return site.cargo

    def take_unknown_recheck(self, source_task):
        """Reserve at most one observation-only retry before unloading."""
        site = self.pending_site(source_task)
        if site.cargo is None:
            raise RuntimeError('PlanD final cargo observation missing')
        if site.cargo.count is not None or site.unknown_recheck_used:
            return False
        site.unknown_recheck_used = True
        return True

    def commit_unload(self, source_task):
        site = self.pending_site(source_task)
        if site.cargo is None:
            raise RuntimeError('PlanD cannot commit an unload without cargo evidence')
        site.deposited_count = site.cargo.count
        site.height_status = 'unknown' if site.cargo.count is None else 'known'
        site.unload_completed_at = time.monotonic()
        site.height_evidence = 'cargo vision plus completed unload; stack geometry unverified'
        return site

    def base_height(self, site_id):
        site = self.sites[site_id]
        return site.deposited_count if site.height_status == 'known' else None

    def mark_topping(self, site_id, status):
        self._check_open()
        if status not in ('done', 'skipped', 'interrupted'):
            raise ValueError('invalid PlanD topping status')
        site = self.sites[site_id]
        if site.topping_status not in ('pending', status):
            raise RuntimeError(f'PlanD site {site_id} topping already finalized')
        site.topping_status = status

    def close(self):
        self.refill_source_task = None
        self.closed = True
