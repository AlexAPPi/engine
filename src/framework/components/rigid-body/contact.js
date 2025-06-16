import { Vec3 } from '../../../core/math/vec3.js'

/**
 * @import { CollisionComponent } from '../collision/component.js'
 * @import { Entity } from '../../entity.js'
 * @import { RigidBodyComponent } from './component.js'
 * @import { RigidBodyComponentSystem } from './system.js'
 */

/**
 * Contains the result of a successful raycast intersection with a rigid body. When a ray
 * intersects with a rigid body in the physics simulation, this class stores the complete
 * information about that intersection including the entity, the exact point of impact, the normal
 * at the impact point, and the fractional distance along the ray where the intersection occurred.
 *
 * Instances of this class are created and returned by {@link RigidBodyComponentSystem#raycastFirst}
 * and {@link RigidBodyComponentSystem#raycastAll} methods when performing physics raycasts.
 *
 * @category Physics
 */
class RaycastResult {
    /**
     * The entity that was hit.
     *
     * @type {Entity}
     */
    entity;

    /**
     * The point at which the ray hit the entity in world space.
     *
     * @type {Vec3}
     */
    point;

    /**
     * The normal vector of the surface where the ray hit in world space.
     *
     * @type {Vec3}
     */
    normal;

    /**
     * The normalized distance (between 0 and 1) at which the ray hit occurred from the
     * starting point.
     *
     * @type {number}
     */
    hitFraction;

    /**
     * Create a new RaycastResult instance.
     *
     * @param {Entity} entity - The entity that was hit.
     * @param {Vec3} point - The point at which the ray hit the entity in world space.
     * @param {Vec3} normal - The normal vector of the surface where the ray hit in world space.
     * @param {number} hitFraction - The normalized distance (between 0 and 1) at which the ray hit
     * occurred from the starting point.
     * @ignore
     */
    constructor(entity, point, normal, hitFraction) {
        this.entity = entity;
        this.point = point;
        this.normal = normal;
        this.hitFraction = hitFraction;
    }
}

/**
 * Represents the detailed data of a single contact point between two rigid bodies in the physics
 * simulation. This class provides comprehensive information about the contact, including the
 * entities involved, the exact contact points in both local and world space coordinates, the
 * contact normal, and the collision impulse force.
 *
 * Instances of this class are created by the physics engine when collision events occur and are
 * passed to event handlers only through the global `contact` event on the
 * {@link RigidBodyComponentSystem}. Individual rigid body components receive instances of
 * {@link ContactResult} instead.
 *
 * @example
 * app.systems.rigidbody.on('contact', (result) => {
 *     console.log(`Contact between ${result.a.name} and ${result.b.name}`);
 * });
 * @category Physics
 */
class SingleContactResult {
    /**
     * The first entity involved in the contact.
     *
     * @type {Entity}
     */
    a;

    /**
     * The second entity involved in the contact.
     *
     * @type {Entity}
     */
    b;

    /**
     * The total accumulated impulse applied by the constraint solver during the last
     * sub-step. Describes how hard two bodies collided.
     *
     * @type {number}
     */
    impulse;

    /**
     * The point on Entity A where the contact occurred, relative to A.
     *
     * @type {Vec3}
     */
    localPointA;

    /**
     * The point on Entity B where the contact occurred, relative to B.
     *
     * @type {Vec3}
     */
    localPointB;

    /**
     * The point on Entity A where the contact occurred, in world space.
     *
     * @type {Vec3}
     */
    pointA;

    /**
     * The point on Entity B where the contact occurred, in world space.
     *
     * @type {Vec3}
     */
    pointB;

    /**
     * The normal vector of the contact on Entity B, in world space.
     *
     * @type {Vec3}
     */
    normal;

    /**
     * Create a new SingleContactResult instance.
     *
     * @param {Entity} a - The first entity involved in the contact.
     * @param {Entity} b - The second entity involved in the contact.
     * @param {ContactPoint} contactPoint - The contact point between the two entities.
     * @ignore
     */
    constructor(a, b, contactPoint) {
        if (arguments.length !== 0) {
            this.a = a;
            this.b = b;
            this.impulse = contactPoint.impulse;
            this.localPointA = contactPoint.localPoint;
            this.localPointB = contactPoint.localPointOther;
            this.pointA = contactPoint.point;
            this.pointB = contactPoint.pointOther;
            this.normal = contactPoint.normal;
        } else {
            this.a = null;
            this.b = null;
            this.impulse = 0;
            this.localPointA = new Vec3();
            this.localPointB = new Vec3();
            this.pointA = new Vec3();
            this.pointB = new Vec3();
            this.normal = new Vec3();
        }
    }
}

/**
 * Represents a single point of contact between two colliding rigid bodies in the physics
 * simulation. Each contact point stores detailed spatial information about the collision,
 * including both local and world space coordinates of the exact contact points on both entities,
 * the contact normal  direction, and the collision impulse force.
 *
 * Contact points are generated by the physics engine during collision detection and are typically
 * accessed through a {@link ContactResult} object, which can contain multiple contact points for a
 * single collision between two entities. Multiple contact points commonly occur when objects
 * collide along edges or faces rather than at a single point.
 *
 * The impulse property can be particularly useful for gameplay mechanics that need to respond
 * differently based on the force of impact, such as damage calculations or sound effect volume.
 *
 * @example
 * // Access contact points from a collision event
 * entity.collision.on('contact', (result) => {
 *     // Get the first contact point
 *     const contact = result.contacts[0];
 *
 *     // Get the contact position in world space
 *     const worldPos = contact.point;
 *
 *     // Check how hard the collision was
 *     if (contact.impulse > 10) {
 *         console.log("That was a hard impact!");
 *     }
 * });
 *
 * @category Physics
 */
class ContactPoint {
    /**
     * The point on the entity where the contact occurred, relative to the entity.
     *
     * @type {Vec3}
     */
    localPoint;

    /**
     * The point on the other entity where the contact occurred, relative to the other entity.
     *
     * @type {Vec3}
     */
    localPointOther;

    /**
     * The point on the entity where the contact occurred, in world space.
     *
     * @type {Vec3}
     */
    point;

    /**
     * The point on the other entity where the contact occurred, in world space.
     *
     * @type {Vec3}
     */
    pointOther;

    /**
     * The normal vector of the contact on the other entity, in world space. This vector points
     * away from the surface of the other entity at the point of contact.
     *
     * @type {Vec3}
     */
    normal;

    /**
     * The total accumulated impulse applied by the constraint solver during the last sub-step.
     * This value represents how hard two objects collided. Higher values indicate stronger impacts.
     *
     * @type {number}
     */
    impulse;

    /**
     * Create a new ContactPoint instance.
     *
     * @param {Vec3} [localPoint] - The point on the entity where the contact occurred, relative to
     * the entity.
     * @param {Vec3} [localPointOther] - The point on the other entity where the contact occurred,
     * relative to the other entity.
     * @param {Vec3} [point] - The point on the entity where the contact occurred, in world space.
     * @param {Vec3} [pointOther] - The point on the other entity where the contact occurred, in
     * world space.
     * @param {Vec3} [normal] - The normal vector of the contact on the other entity, in world
     * space.
     * @param {number} [impulse] - The total accumulated impulse applied by the constraint solver
     * during the last sub-step. Describes how hard two objects collide. Defaults to 0.
     * @ignore
     */
    constructor(localPoint = new Vec3(), localPointOther = new Vec3(), point = new Vec3(), pointOther = new Vec3(), normal = new Vec3(), impulse = 0) {
        this.localPoint = localPoint;
        this.localPointOther = localPointOther;
        this.point = point;
        this.pointOther = pointOther;
        this.normal = normal;
        this.impulse = impulse;
    }
}

/**
 * Represents a collection of contact points between two entities in a physics collision.
 * When rigid bodies collide, this object stores the entity involved in the collision and
 * an array of specific contact points where the collision occurred. This information is
 * used by the physics system to resolve collisions and notify components through events.
 *
 * Instances of this class are passed to event handlers for the `contact` and `collisionstart`
 * events on individual {@link RigidBodyComponent} and {@link CollisionComponent} instances.
 *
 * Unlike {@link SingleContactResult} which is used for global contact events, ContactResult
 * objects provide information about collision from the perspective of one entity, with
 * information about which other entity was involved and all points of contact.
 *
 * Please refer to the following event documentation for more information:
 *
 * - {@link CollisionComponent.EVENT_CONTACT}
 * - {@link CollisionComponent.EVENT_COLLISIONSTART}
 * - {@link RigidBodyComponent.EVENT_CONTACT}
 * - {@link RigidBodyComponent.EVENT_COLLISIONSTART}
 *
 * @category Physics
 */
class ContactResult {
    /**
     * The entity that was involved in the contact with this entity.
     *
     * @type {Entity}
     */
    other;

    /**
     * An array of ContactPoints with the other entity.
     *
     * @type {ContactPoint[]}
     */
    contacts;

    /**
     * Create a new ContactResult instance.
     *
     * @param {Entity} other - The entity that was involved in the contact with this entity.
     * @param {ContactPoint[]} contacts - An array of ContactPoints with the other entity.
     * @ignore
     */
    constructor(other, contacts) {
        this.other = other;
        this.contacts = contacts;
    }
}

export { ContactPoint, ContactResult, RaycastResult, SingleContactResult };
