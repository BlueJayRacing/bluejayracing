import {SpringOptns, Triplet} from "@react-three/cannon";

export type SusLinkInfo = {
    length: number,
    width: number,
    height: number,
    mass: number,
    bodyAttachPoint: Triplet, // in body local coords
    springAttachPoint?: Triplet // in link local coords (optional)
}

export type UprightInfo = {
    length: number,
    width: number,
    height: number,
    mass: number,
    topAttachPoint: Triplet // in local coords
    bottomAttachPoint: Triplet // in local coords
    wheelAttachPoint: Triplet
}

export type SpringInfo = {
    stiffness: number,
    restLength: number,
    damping: number,
    bodyAnchor: Triplet
    susAnchor: Triplet
}

export type WheelInfo = {
    radius: number,
    width: number,
    mass: number
}
