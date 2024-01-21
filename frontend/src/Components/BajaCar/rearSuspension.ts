import {RefObject, useRef} from "react";
import {Object3D} from "three";
import {SpringOptns, Triplet, useBox, useCylinder, useHingeConstraint, useSpring} from "@react-three/cannon";
import {SpringInfo, SusLinkInfo, UprightInfo, WheelInfo} from "./bajaCarTypes";

export function makeRearSuspension(chassisBody: RefObject<Object3D>, position: Triplet) {
    const width = 0.6;
    const height = 1;
    const length = 1.6;

    let susInfoBottom: SusLinkInfo = {
        bodyAttachPoint: [width / 2, -height / 2 + 0.2, -0.6],
        length: 0.5,
        width: 0.1,
        height: 0.05,
        mass: 5,
    }

    let susInfoTop: SusLinkInfo = {
        bodyAttachPoint: [width / 2, -height / 2 + 0.4, -0.6],
        length: 0.56,
        width: 0.1,
        height: 0.05,
        mass: 5,
        springAttachPoint: [0.1, 0.05, 0]
    }

    let uprInfo: UprightInfo = {
        length: 0.1,
        width: 0.08,
        height: 0.2,
        mass: 1,
        topAttachPoint: [0, 0.05, 0], // in local coords
        bottomAttachPoint: [0, -0.05, 0],// in local coords
        wheelAttachPoint: [0, -0.05, 0]
    }

    let springInfo: SpringInfo = {
        stiffness: 5000,
        restLength: 0.7,
        damping: 100,
        bodyAnchor: [width / 2, 0.3, -0.6],
        susAnchor: susInfoTop.springAttachPoint,
    }
    let wheelInfo: WheelInfo = {
        radius: 0.2,
        width: 0.1,
        mass: 2
    }

    let [susLinkBottom, SusApiBottom, susConstraintBottom] = makeSusLink(susInfoBottom, chassisBody, position)
    let [susLinkTop, susApiTop, susConstraintTop] = makeSusLink(susInfoTop, chassisBody, position)
    let [upr, uprApi, topUprConstraint, bottomUprConstraint] = makeUpright(uprInfo, susLinkTop, susInfoTop, susLinkBottom, susInfoBottom)
    let springApi = makeSpring(springInfo, chassisBody, susLinkTop)
    let [wheel, wheelApi, wheelConstraint] = makeWheel(wheelInfo, uprInfo, upr)

    let [susLinkBottomMir, SusApiBottomMir, susConstraintBottomMir] = makeSusLink(susInfoBottom, chassisBody, position, true)
    let [susLinkTopMir, susApiTopMir, susConstraintTopMir] = makeSusLink(susInfoTop, chassisBody, position, true)
    let [uprMir, uprApiMir, topUprConstraintMir, bottomUprConstraintMir] = makeUpright(uprInfo, susLinkTopMir, susInfoTop, susLinkBottomMir, susInfoBottom, true)
    let springApiMir = makeSpring(springInfo, chassisBody, susLinkTopMir, true)
    let [wheelMir, wheelApiMir, wheelConstraintMir] = makeWheel(wheelInfo, uprInfo, uprMir, true)
    return [wheelApi, wheelApiMir]
}

export function makeSusLink(susInfo: SusLinkInfo, body: RefObject<Object3D>, bodyPos: Triplet, mirrored = false) {
    let [sus, susApi] = useBox(() => ({
        args: [susInfo.length, susInfo.height, susInfo.width],
        mass: susInfo.mass,
        position: [0, 0, 0],
    }), useRef(null));

    let susPivot: Triplet = [-susInfo.length / 2, 0, 0]
    let susConstraint = useHingeConstraint(body, sus, {
        pivotA: mirrored ? mirrorTriplet(susInfo.bodyAttachPoint, 'x') : susInfo.bodyAttachPoint,
        axisA: [0, 0, 1],
        pivotB: mirrored ? mirrorTriplet(susPivot, 'x') : susPivot,
        axisB: [0, 0, 1],
        maxForce: 1e18
    })
    return [sus, susApi, susConstraint]
}

export function makeUpright(uprInfo: UprightInfo, topSus: RefObject<Object3D>, topSusInfo: SusLinkInfo, bottomSus: RefObject<Object3D>, bottomSusInfo: SusLinkInfo, mirrored = false) {
    let [upr, uprApi] = useBox(() => ({
        args: [uprInfo.width, uprInfo.height, uprInfo.length],
        mass: uprInfo.mass,
        position: [0, 0, 0],
    }), useRef(null));
    let topSusAttach: Triplet = [topSusInfo.length / 2, 0, 0]
    let topConstraint = useHingeConstraint(upr, topSus, {
        pivotA: uprInfo.topAttachPoint,
        axisA: [0, 0, 1],
        pivotB: mirrored ? mirrorTriplet(topSusAttach, 'x') : topSusAttach,
        axisB: [0, 0, 1],
        maxForce: 1e18
    })
    let bottomSusAttach: Triplet = [bottomSusInfo.length / 2, 0, 0]
    let bottomConstraint = useHingeConstraint(upr, bottomSus, {
        pivotA: uprInfo.bottomAttachPoint,
        axisA: [0, 0, 1],
        pivotB: mirrored ? mirrorTriplet(bottomSusAttach, 'x') : bottomSusAttach,
        axisB: [0, 0, 1],
        maxForce: 1e18

    })


    return [upr, uprApi, topConstraint, bottomConstraint]
}

export function makeSpring(springInfo: SpringInfo, chassisBody: RefObject<Object3D>, susLinkTop: RefObject<Object3D>, mirrored = false) {
    let springOptns: SpringOptns = {
        stiffness: springInfo.stiffness,
        restLength: springInfo.restLength,
        damping: springInfo.damping,
        localAnchorA: mirrored ? mirrorTriplet(springInfo.bodyAnchor, 'x') : springInfo.bodyAnchor,
        localAnchorB: mirrored ? mirrorTriplet(springInfo.susAnchor, 'x') : springInfo.susAnchor
    }

    let [, , springApi] = useSpring(chassisBody, susLinkTop, springOptns)
    return springApi

}

export function makeWheel(wheelInfo: WheelInfo, uprInfo: UprightInfo, upr: RefObject<Object3D>, mirrored = false) {

    let [wheel, wheelApi] = useCylinder(() => ({
            args: [wheelInfo.radius, wheelInfo.radius, wheelInfo.width, 64],
            mass: wheelInfo.mass,
            position: [0, 0, 0],
            rotation: [0, 0, Math.PI / 2]
        }
    ), useRef(null))

    let wheelPivot: Triplet = [0, wheelInfo.width / 2, 0]
    let wheelConstraint = useHingeConstraint(upr, wheel, {
        pivotA: mirrored ? mirrorTriplet(uprInfo.wheelAttachPoint, 'x') : uprInfo.wheelAttachPoint,
        axisA: [1, 0, 0],
        pivotB: mirrored ? mirrorTriplet(wheelPivot, 'y') : wheelPivot,
        axisB: [0, 1, 0],
        maxForce: 1e24
    })
    return [wheel, wheelApi, wheelConstraint]
}
