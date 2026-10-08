"""The pens in: one per slot (gel in one row, pencils in another), kept by the rig in
`config/<rig>/pens.json` (`Rig.write_pens_in`).  `GET /pens` lists them; `POST /pens/{slot}`
{"name": ...} puts another pen into a slot's holder and reloads the rig — refused while a job
runs (it plans with the pens in) and for a pen the rig's table does not have.
"""
from __future__ import annotations

from aris.types import Refusal


def table(rig) -> list[str]:
    """The pens the rig knows: its pens table's names when the rig shows them (`pen_table`),
    else the pens that are in and the default pen (`write_pens_in` refuses any other)."""
    t = getattr(rig, "pen_table", None)
    if isinstance(t, dict):
        return sorted(t)
    return sorted(set(rig.pens_in.values()) | {rig.pen_name})


def view(st) -> dict:
    return dict(pens_in={a: st.pen(a).get("name") for a in st.rig.arm_ids},
                table=table(st.rig))


def put_in(st, store, slot: str, name: str) -> dict | Refusal:
    from aris.rig import Rig
    if slot not in st.rig.arm_ids:
        return Refusal("no_slot", f"no slot {slot} on this rig ({', '.join(st.rig.arm_ids)})")
    busy = store.running() if hasattr(store, "running") else None
    if busy is not None:
        return Refusal("busy", f"job {busy.id} is {busy.state}: change pens between jobs")
    try:
        Rig.write_pens_in(st.config_dir, {slot: name})
    except (ValueError, KeyError) as e:
        return Refusal("no_pen", str(e))
    st.reload()
    return view(st)
