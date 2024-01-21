import {Triplet} from "@react-three/cannon";
import {Vector3} from "three";

export type Axis = 'x' | 'y' | 'z'

export function mirrorTriplet(tr: Triplet, ax: Axis): Triplet {
    switch (ax) {
        case 'x':
            return [-tr[0], tr[1], tr[2]]
        case 'y':
            return [tr[0], -tr[1], tr[2]]
        case 'z':
            return [tr[0], tr[1], -tr[2]]
    }
}

export function addTriplets(lhs: Triplet, rhs: Triplet): Triplet {
    return [lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]];
}

export function computeAxis(a: Vector3, b: Vector3): Vector3 {
    let dir = new Vector3()
    dir.subVectors(b, a)
    return dir.normalize()
}

// https://math.stackexchange.com/questions/4347497/find-a-point-on-a-line-that-creates-a-perpendicular-in-3d-space
export function computeTriangleIntersect(a: Vector3, b: Vector3, c: Vector3): Vector3 {
    let dir = new Vector3()
    dir.subVectors(c, a)
    let bSubA = new Vector3()
    bSubA.subVectors(b, a)
    let t = bSubA.dot(dir) / dir.dot(dir)
    console.log(t)
    return a.add(dir.multiplyScalar(t))
}

export function toTriplet(v: Vector3): Triplet {
    return [v.x, v.y, v.z]
}