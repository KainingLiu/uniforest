"""Orange collection contract, mirrored in the independently deployed game package."""


class BothOrangeAreasExhausted(RuntimeError):
    """Both searches ended; the next competition decision is intentionally pending."""

    def __init__(self, count):
        self.count = checked_count(count)
        self.status = 'both_areas_exhausted_pending'
        super().__init__(f'both orange areas exhausted with {count}/3 cubes; next decision pending')


def checked_count(count):
    """Unknown observation must never be treated as either zero or a full load."""
    if type(count) is not int or not 0 <= count <= 3:
        raise RuntimeError('orange collection requires a fresh, known cargo count (0..3)')
    return count


def missing(count):
    return 3 - checked_count(count)


def should_change_region(count, *, exhausted):
    return missing(count) > 0 and exhausted
