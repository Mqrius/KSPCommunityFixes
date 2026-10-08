using HarmonyLib;
using System;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace KSPCommunityFixes.Performance
{
    /// <summary>
    /// Map view and tracking station redraw every orbit line every frame (resample, convert to scaled space, rebuild
    /// the line mesh), even though an on-rails orbit doesn't change between frames. This skips the redraw of a line
    /// when nothing it depends on changed since its last real redraw : orbit elements, reference body, scaled space
    /// offset, camera pose / fov / screen size, line color (including fade opacity), active state, and the vessel
    /// position along the orbit (quantized, it only drives the fade gradient). Any change redraws exactly like stock.
    /// A pure change of the scaled space offset (map focus moving) just moves the already baked 3D line.
    /// </summary>
    class OrbitLineCache : BasePatch
    {
        private const double MaxTranslateRatio = 0.02;

        private sealed class State
        {
            public bool valid;
            public double sma, ecc, inc, lpe, lan, mna, epoch;
            public CelestialBody body;
            public Vector3d shift;
            public Vector3 camPos;
            public Quaternion camRot;
            public float fov;
            public int screenW, screenH, eccQ;
            public Color color;
            public bool active, draw3d;
        }

        private static readonly ConditionalWeakTable<OrbitRendererBase, State> states = new ConditionalWeakTable<OrbitRendererBase, State>();
        private static readonly State current = new State();

        private static Func<OrbitRendererBase, Color> getOrbitColour;
        private static Action<OrbitRendererBase> splineOpacityUpdate;
        private static AccessTools.FieldRef<OrbitRendererBase, float> lineOpacity;
        private static AccessTools.FieldRef<OrbitRendererBase, bool> isFocused;

        // set when the original DrawOrbit will run after our prefix, so that it doesn't step the fade a second time
        private static bool suppressOpacityUpdate;

        // state to validate once the original has redrawn the line
        private static State pending;

        protected override void ApplyPatches()
        {
            var drawOrbit = AccessTools.Method(typeof(OrbitRendererBase), "DrawOrbit");
            var opacityUpdate = AccessTools.Method(typeof(OrbitRendererBase), "SplineOpacityUpdate");
            var orbitColour = AccessTools.Method(typeof(OrbitRendererBase), "GetOrbitColour");

            getOrbitColour = (Func<OrbitRendererBase, Color>)Delegate.CreateDelegate(typeof(Func<OrbitRendererBase, Color>), orbitColour);
            splineOpacityUpdate = (Action<OrbitRendererBase>)Delegate.CreateDelegate(typeof(Action<OrbitRendererBase>), opacityUpdate);
            lineOpacity = AccessTools.FieldRefAccess<OrbitRendererBase, float>("lineOpacity");
            isFocused = AccessTools.FieldRefAccess<OrbitRendererBase, bool>(nameof(OrbitRendererBase.isFocused));

            AddPatch(PatchType.Prefix, drawOrbit, nameof(OrbitRendererBase_DrawOrbit_Prefix), Priority.First);
            AddPatch(PatchType.Postfix, drawOrbit, nameof(OrbitRendererBase_DrawOrbit_Postfix));
            AddPatch(PatchType.Prefix, opacityUpdate, nameof(OrbitRendererBase_SplineOpacityUpdate_Prefix), Priority.First);
        }

        static bool OrbitRendererBase_DrawOrbit_Prefix(OrbitRendererBase __instance, OrbitRendererBase.DrawMode mode)
        {
            State cached = states.GetOrCreateValue(__instance);
            pending = null;

            // the original runs this first, we do it here so that its result is part of the key
            splineOpacityUpdate(__instance);
            suppressOpacityUpdate = true;

            if (lineOpacity(__instance) <= 0f || !Snapshot(__instance, mode, current))
            {
                cached.valid = false;
                return true;
            }

            if (cached.valid && Matches(cached, current, out bool translate))
            {
                suppressOpacityUpdate = false;
                if (translate)
                    __instance.OrbitLine.rectTransform.position = (Vector3)(current.shift - cached.shift);
                return false;
            }

            Copy(current, cached);
            cached.valid = false;
            pending = cached;
            return true;
        }

        static void OrbitRendererBase_DrawOrbit_Postfix()
        {
            suppressOpacityUpdate = false;
            if (pending != null)
                pending.valid = true;
            pending = null;
        }

        static bool OrbitRendererBase_SplineOpacityUpdate_Prefix()
        {
            if (!suppressOpacityUpdate)
                return true;

            suppressOpacityUpdate = false;
            return false;
        }

        // false when the line must always be redrawn (not a plain redraw, focused, no camera...)
        private static bool Snapshot(OrbitRendererBase renderer, OrbitRendererBase.DrawMode mode, State s)
        {
            Camera cam = PlanetariumCamera.Camera;
            Orbit orbit = renderer.driver?.orbit;

            if (mode != OrbitRendererBase.DrawMode.REDRAW_AND_RECALCULATE
                || renderer.OrbitLine == null
                || isFocused(renderer)
                || cam == null
                || orbit?.referenceBody == null)
                return false;

            s.sma = orbit.semiMajorAxis; s.ecc = orbit.eccentricity; s.inc = orbit.inclination; s.lpe = orbit.argumentOfPeriapsis;
            s.lan = orbit.LAN; s.mna = orbit.meanAnomalyAtEpoch; s.epoch = orbit.epoch; s.body = orbit.referenceBody;
            s.shift = orbit.referenceBody.position * ScaledSpace.InverseScaleFactor + ScaledSpace.LocalToScaledSpace(Vector3d.zero);

            Transform camTransform = cam.transform;
            s.camPos = camTransform.position; s.camRot = camTransform.rotation; s.fov = cam.fieldOfView;
            s.screenW = Screen.width; s.screenH = Screen.height;

            double revolutions = orbit.eccentricity < 1.0 ? (((orbit.eccentricAnomaly % UtilMath.TwoPI) + UtilMath.TwoPI) % UtilMath.TwoPI) / UtilMath.TwoPI : 0.0;
            s.eccQ = (int)(revolutions * 512.0);

            s.color = getOrbitColour(renderer);
            s.active = renderer.OrbitLine.active;
            s.draw3d = MapView.Draw3DLines;
            return true;
        }

        // translate is set when the line can be reused by only moving its transform (scaled space offset changed, but
        // not so much that the billboard width would visibly drift)
        private static bool Matches(State a, State b, out bool translate)
        {
            translate = false;

            if (a.sma != b.sma || a.ecc != b.ecc || a.inc != b.inc || a.lpe != b.lpe || a.lan != b.lan || a.mna != b.mna || a.epoch != b.epoch || a.body != b.body)
                return false;

            Vector3d shiftDelta = b.shift - a.shift;
            if (shiftDelta.sqrMagnitude >= 1e-12)
            {
                if (!a.draw3d || shiftDelta.magnitude > MaxTranslateRatio * (b.camPos - (Vector3)b.shift).magnitude)
                    return false;
                translate = true;
            }

            return a.camPos == b.camPos
                && a.camRot == b.camRot
                && a.fov == b.fov && a.screenW == b.screenW && a.screenH == b.screenH
                && a.eccQ == b.eccQ
                && ColorNear(a.color, b.color)
                && a.active == b.active && a.draw3d == b.draw3d;
        }

        // within 1/64 per channel : far below a visible step
        private static bool ColorNear(Color a, Color b)
        {
            return Mathf.Abs(a.r - b.r) < 0.016f && Mathf.Abs(a.g - b.g) < 0.016f
                && Mathf.Abs(a.b - b.b) < 0.016f && Mathf.Abs(a.a - b.a) < 0.016f;
        }

        private static void Copy(State from, State to)
        {
            to.sma = from.sma; to.ecc = from.ecc; to.inc = from.inc; to.lpe = from.lpe; to.lan = from.lan; to.mna = from.mna; to.epoch = from.epoch;
            to.body = from.body; to.shift = from.shift; to.camPos = from.camPos; to.camRot = from.camRot; to.fov = from.fov;
            to.screenW = from.screenW; to.screenH = from.screenH; to.eccQ = from.eccQ; to.color = from.color;
            to.active = from.active; to.draw3d = from.draw3d;
        }
    }
}
