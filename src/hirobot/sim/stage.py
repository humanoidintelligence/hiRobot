"""A default stage (floor, light, sky) for a simulation world whose models bring none.

Most robot MJCFs (MuJoCo Menagerie arms, the Unitree H1 / H2 files, URDFs) are the
robot alone: no floor, no light, no sky. Rendered as is, a camera looking past the
robot sees black, so a head camera on such a model shows nothing. The Unitree G1
files carry their own floor and light; a world built from them is left as it is.
"""

from __future__ import annotations

from typing import Any

FLOOR = "hirobot/floor"


def _has_floor(mj: Any, spec: Any) -> bool:
    return any(g.type == mj.mjtGeom.mjGEOM_PLANE for g in spec.geoms)


def _has_sky(mj: Any, spec: Any) -> bool:
    return any(t.type == mj.mjtTexture.mjTEXTURE_SKYBOX for t in spec.textures)


def floor_height(mj: Any, spec: Any) -> float:
    """Where the floor goes: the lowest point of the models in their initial pose, or 0.

    A humanoid welded at its root's height (or a URDF, its root at the origin) has its
    feet at that point; an arm's base sits at 0 and its links above.
    """
    import numpy as np

    m = spec.compile()
    d = mj.MjData(m)
    mj.mj_forward(m, d)
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    low = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == 0 or m.geom_type[g] == mj.mjtGeom.mjGEOM_PLANE:
            continue
        centre, half = m.geom_aabb[g, :3], m.geom_aabb[g, 3:]
        corners = (centre + signs * half) @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        low.append(float(corners[:, 2].min()))
    return round(min(0.0, min(low)), 4) if low else 0.0


def add_default_stage(mj: Any, spec: Any) -> None:
    """Add a floor, a light and a sky to ``spec``, each only if it has none.

    The floor is a checkered plane in the world body under the models' lowest point
    (:func:`floor_height`), drawn only: it takes part in no contacts, so the physics
    (and the simulation check) are those of the models. The light is a directional
    light from above, with shadows; the sky a soft blue gradient.
    """
    if not _has_sky(mj, spec):
        sky = spec.add_texture()
        sky.name = "hirobot/sky"
        sky.type = mj.mjtTexture.mjTEXTURE_SKYBOX
        sky.builtin = mj.mjtBuiltin.mjBUILTIN_GRADIENT
        sky.rgb1 = [0.55, 0.68, 0.82]
        sky.rgb2 = [0.12, 0.16, 0.22]
        sky.width, sky.height = 512, 3072
    if not _has_floor(mj, spec):
        tex = spec.add_texture()
        tex.name = "hirobot/floor"
        tex.type = mj.mjtTexture.mjTEXTURE_2D
        tex.builtin = mj.mjtBuiltin.mjBUILTIN_CHECKER
        tex.rgb1 = [0.36, 0.40, 0.45]
        tex.rgb2 = [0.26, 0.29, 0.33]
        tex.mark = mj.mjtMark.mjMARK_EDGE
        tex.markrgb = [0.6, 0.6, 0.6]
        tex.width, tex.height = 300, 300
        mat = spec.add_material()
        mat.name = "hirobot/floor"
        textures = list(mat.textures)
        textures[mj.mjtTextureRole.mjTEXROLE_RGB] = "hirobot/floor"
        mat.textures = textures
        mat.texrepeat = [5, 5]
        mat.texuniform = True
        mat.reflectance = 0.1
        z = floor_height(mj, spec)
        floor = spec.worldbody.add_geom()
        floor.name = FLOOR
        floor.type = mj.mjtGeom.mjGEOM_PLANE
        floor.pos = [0, 0, z]
        floor.size = [0, 0, 0.05]  # 0 x 0: an infinite plane
        floor.material = "hirobot/floor"
        # Drawn only: no contacts. An arm on a table reaches below its base plane, and the
        # simulation check (sim_check) reads any contact with a world geom as "into the
        # ground"; a colliding floor would refuse those motions. Physics stays as the
        # models define it.
        floor.contype = 0
        floor.conaffinity = 0
    if not spec.lights:
        light = spec.worldbody.add_light()
        light.name = "hirobot/light"
        light.pos = [0, 0, 4]
        light.dir = [0, 0, -1]
        light.type = mj.mjtLightType.mjLIGHT_DIRECTIONAL
        light.diffuse = [0.7, 0.7, 0.7]
        light.specular = [0.2, 0.2, 0.2]
        light.castshadow = True
        h = spec.visual.headlight
        h.ambient = [0.3, 0.3, 0.3]
        h.diffuse = [0.4, 0.4, 0.4]
