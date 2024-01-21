import {
    Triplet,
    useBox, useCylinder, useHingeConstraint, usePointToPointConstraint
} from "@react-three/cannon"
import {SpringInfo, SusLinkInfo, UprightInfo, WheelInfo} from "./bajaCarTypes";
import {RefObject, useRef} from "react";
import {Object3D, Vector3} from "three";
import {computeAxis, computeTriangleIntersect, toTriplet} from "./susUtils";
import {Simulate} from "react-dom/test-utils";


type VWishboneInfoComputed = { innerHingePoint: Vector3, innerAxis: Vector3, length: number, }
type VWishboneInfo = VWishboneInfoComputed & { width: number, height: number, mass: number }

type VUprightInfoComputed = { height: number, axlePoint: Vector3, tieRodPoint: Vector3 }
type VUprightInfo = VUprightInfoComputed & { length: number, width: number, mass: number }

type VWheelInfo = { radius: number, width: number, mass: number }

type VTieRodInfoComputed = { length: number, innerPoint: Vector3 }
type VTieRodInfo = VTieRodInfoComputed & { radius: number, mass: number }

function computeFrontWishbone(frontInner: Vector3, rearInner: Vector3, outerBall: Vector3): VWishboneInfoComputed {
    // inner axis direction on body (normalized)
    let innerAxis = computeAxis(frontInner, rearInner)
    console.log(innerAxis)

    // inner virtual hinge point
    console.log(frontInner, rearInner)
    let innerHingePoint = computeTriangleIntersect(frontInner, outerBall, rearInner)

    // length of the virtual link, in meters
    let linkLength = innerHingePoint.distanceTo(outerBall) / 1000

    // Convert inner hinge to meters
    innerHingePoint.divideScalar(1000)

    return {innerHingePoint, innerAxis, length: linkLength}
}


export function computeFrontUpright(lowerBall: Vector3, upperBall: Vector3, wheelSpindlePos: Vector3, tieRodPos: Vector3): VUprightInfoComputed {
    // vector going from lower to upper ball, normalized
    let uprDir = new Vector3()
    uprDir.subVectors(upperBall, lowerBall)
    let height = uprDir.length() / 1000
    console.log(height)

    // origin of upright
    let centerPt = new Vector3()
    centerPt.copy(lowerBall)
    centerPt.addScaledVector(uprDir, 0.5)

    let axlePoint = new Vector3()
    axlePoint.subVectors(wheelSpindlePos, centerPt)
    axlePoint.divideScalar(1000)
    let tieRodPoint = new Vector3()
    tieRodPoint.subVectors(tieRodPos, centerPt)
    tieRodPoint.divideScalar(1000)

    return {height, axlePoint, tieRodPoint}
}

function computeTieRod(tieInnerPt: Vector3, tieOuterPt: Vector3): VTieRodInfoComputed {

    let length = tieInnerPt.distanceTo(tieOuterPt)
    length /= 1000
    let innerPoint = new Vector3()
    innerPoint.copy(tieInnerPt)
    innerPoint.divideScalar(1000)

    return {length, innerPoint}
}

export function makeFrontWishbone(wishInfo: VWishboneInfo, body: RefObject<Object3D>, mirrored = false) {
    let [wish, wishApi] = useBox(() => ({
        args: [wishInfo.length, wishInfo.width, wishInfo.height],
        mass: wishInfo.mass,
        position: [0, 0, 0],
    }), useRef(null));

    let hingePivot: Triplet = [-wishInfo.length / 2, 0, 0]
    let wishConstraint = useHingeConstraint(body, wish, {
        pivotA: toTriplet(wishInfo.innerHingePoint),
        axisA: toTriplet(wishInfo.innerAxis),
        pivotB: hingePivot,
        axisB: [0, 0, 1],
        maxForce: 1e18
    })

    return [wish, wishApi, wishConstraint]
}

function makeFrontUpright(uprInfo: VUprightInfo, upperWish: RefObject<Object3D>, upperWishInfo: VWishboneInfo, lowerWish: RefObject<Object3D>, lowerWishInfo: VWishboneInfo) {
    let [upr, uprApi] = useBox(() => ({
        args: [uprInfo.length, uprInfo.height, uprInfo.width],
        mass: uprInfo.mass,
        position: [0, 0, 0],
    }), useRef(null));

    let upperConstraint = usePointToPointConstraint(upperWish, upr, {
        pivotA: [upperWishInfo.length / 2, 0, 0],
        pivotB: [0, uprInfo.height / 2, 0],
        maxForce: 1e18
    })
    let lowerConstraint = usePointToPointConstraint(lowerWish, upr, {
        pivotA: [lowerWishInfo.length / 2, 0, 0],
        pivotB: [0, -uprInfo.height / 2, 0],
        maxForce: 1e18
    })

    return [upr, uprApi]
}

export function makeWheel(wheelInfo: VWheelInfo, uprInfo: VUprightInfo, upr: RefObject<Object3D>, mirrored = false) {

    let [wheel, wheelApi] = useCylinder(() => ({
            args: [wheelInfo.radius, wheelInfo.radius, wheelInfo.width, 64],
            mass: wheelInfo.mass,
            position: [0, 0, 0],
            rotation: [0, 0, Math.PI / 2]
        }
    ), useRef(null))

    let wheelPivot: Triplet = [0, wheelInfo.width / 2, 0]
    let wheelConstraint = useHingeConstraint(upr, wheel, {
        pivotA: toTriplet(uprInfo.axlePoint),
        axisA: [1, 0, 0],
        pivotB: wheelPivot,
        axisB: [0, 1, 0],
        maxForce: 1e24
    })
    return [wheel, wheelApi, wheelConstraint]
}


function makeTieRod(tieRodInfo: VTieRodInfo, uprInfo: VUprightInfo, upr: RefObject<Object3D>, body: RefObject<Object3D>) {
    let [tieRod, tieRodApi] = useCylinder(() => ({
            args: [tieRodInfo.radius, tieRodInfo.radius, tieRodInfo.length, 16],
            mass: tieRodInfo.mass,
            position: [0, 0, 0],
            rotation: [0, 0, Math.PI / 2]
        }
    ), useRef(null))

    let outerConstraint = usePointToPointConstraint(tieRod, upr, {
        pivotA: [0, -tieRodInfo.length / 2, 0],
        pivotB: toTriplet(uprInfo.tieRodPoint),
        maxForce: 1e18
    })
    let innerConstraint = usePointToPointConstraint(tieRod, body, {
        pivotA: [0, tieRodInfo.length / 2, 0],
        pivotB: toTriplet(tieRodInfo.innerPoint),
        maxForce: 1e18
    })

    return [tieRod, tieRodApi, outerConstraint, innerConstraint]

}

export function makeFrontSuspension(chassisBody: RefObject<Object3D>) {

    let frontWishboneWidth = 0.05
    let frontWishboneHeight = 0.05
    let lowerFrontInner = new Vector3(143.215, 59.125, 144.741)
    let lowerRearInner = new Vector3(143.215, 0.021, 448.804)
    let lowerOuterBall = new Vector3(597.489, -130.26, 284.359)

    let upperFrontInner = new Vector3(162.527, 168.196, 100.192)
    let upperRearInner = new Vector3(162.527, 102.167, 439.878)
    let upperOuterBall = new Vector3(576.606, -0.078, 309.874)
    let wheelSpindlePoint = new Vector3(620, -58.964, 265.308)
    let tieRodOuterPoint = new Vector3(580.9, 25.589, 262.956)
    let tieRodInnerPoint = new Vector3(184.523, 149.2, 246.496)

    let vLowerWishInfo: VWishboneInfo = {
        ...computeFrontWishbone(lowerFrontInner, lowerRearInner, lowerOuterBall),
        width: 0.05,
        height: 0.05,
        mass: 1,
    }
    let vUpperWishInfo: VWishboneInfo = {
        ...computeFrontWishbone(upperFrontInner, upperRearInner, upperOuterBall),
        width: 0.05,
        height: 0.05,
        mass: 1,
    }

    let [lowerWish, lowerWishApi, lowerWishConstraint] = makeFrontWishbone(vLowerWishInfo, chassisBody)
    let [upperWish, upperWishApi, upperWishConstraint] = makeFrontWishbone(vUpperWishInfo, chassisBody)

    let vUprInfo: VUprightInfo = {
        ...computeFrontUpright(lowerOuterBall, upperOuterBall, wheelSpindlePoint, tieRodOuterPoint),
        length: 0.05,
        width: 0.05,
        mass: 1
    }

    let [upr, uprApi, upperUprConstraint, lowerUprConstraint] = makeFrontUpright(vUprInfo, upperWish, vUpperWishInfo, lowerWish, vLowerWishInfo)

    let vTieRodInfo: VTieRodInfo = {
        ...computeTieRod(tieRodInnerPoint, tieRodOuterPoint),
        radius: 0.02,
        mass:0.5
    }

    let [tieRod, tieRodApi, outerConstraint, innerConstraint] = makeTieRod(vTieRodInfo, vUprInfo, upr, chassisBody)

    let vWheelinfo: VWheelInfo = {
        radius: 0.25,
        width: 0.15,
        mass: 2
    }
    makeWheel(vWheelinfo, vUprInfo, upr)
}
