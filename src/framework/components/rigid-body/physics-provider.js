import { ObjectPool } from '../../../core/object-pool.js';
import { ContactPoint, ContactResult, SingleContactResult } from './contact.js';

/**
 * @import { Entity } from '../../entity.js'
 * @import { RigidBodyComponentSystem } from './system.js';
 */

class PhysicsProvider {

    /**
     * 
     * @param {RigidBodyComponentSystem} system 
     */
    constructor(system) {
        
        this.system = system;
        this.contactPointPool = new ObjectPool(ContactPoint, 1);
        this.contactResultPool = new ObjectPool(ContactResult, 1);
        this.singleContactResultPool = new ObjectPool(SingleContactResult, 1);

        this.collisions = {};
        this.frameCollisions = {};
    }

    /**
     * 
     * @param {Entity} a 
     * @param {Entity} b 
     * @param {ContactPoint} contactPoint 
     * @returns 
     */
    _createSingleContactResult(a, b, contactPoint) {
        const result = this.singleContactResultPool.allocate();

        result.a = a;
        result.b = b;
        result.localPointA = contactPoint.localPoint;
        result.localPointB = contactPoint.localPointOther;
        result.pointA = contactPoint.point;
        result.pointB = contactPoint.pointOther;
        result.normal = contactPoint.normal;
        result.impulse = contactPoint.impulse;

        return result;
    }
    
    /**
     * 
     * @param {Entity} other
     * @param {ContactPoint[]} contacts 
     * @returns 
     */
    _createContactResult(other, contacts) {
        const result = this.contactResultPool.allocate();
        result.other = other;
        result.contacts = contacts;
        return result;
    }

    /**
     * Returns true if the entity has a contact event attached and false otherwise.
     *
     * @param {Entity} entity - Entity to test.
     * @returns {boolean} True if the entity has a contact and false otherwise.
     * @protected
     */
    _hasContactEvent(entity) {
        const c = entity.collision;
        if (c && (c.hasEvent('collisionstart') || c.hasEvent('collisionend') || c.hasEvent('contact'))) {
            return true;
        }

        const r = entity.rigidbody;
        return r && (r.hasEvent('collisionstart') || r.hasEvent('collisionend') || r.hasEvent('contact'));
    }

    /**
     * Stores a collision between the entity and other in the contacts map and returns true if it
     * is a new collision.
     *
     * @param {Entity} entity - The entity.
     * @param {Entity} other - The entity that collides with the first entity.
     * @returns {boolean} True if this is a new collision, false otherwise.
     * @protected
     */
    _storeCollision(entity, other) {
        let isNewCollision = false;
        const guid = entity.getGuid();

        this.collisions[guid] = this.collisions[guid] || { others: [], entity: entity };

        if (this.collisions[guid].others.indexOf(other) < 0) {
            this.collisions[guid].others.push(other);
            isNewCollision = true;
        }

        this.frameCollisions[guid] = this.frameCollisions[guid] || { others: [], entity: entity };
        this.frameCollisions[guid].others.push(other);

        return isNewCollision;
    }

    /**
     * Removes collisions that no longer exist from the collisions list and fires collisionend
     * events to the related entities.
     *
     * @protected
     */
    _cleanOldCollisions() {
        for (const guid in this.collisions) {
            if (this.collisions.hasOwnProperty(guid)) {
                const frameCollision = this.frameCollisions[guid];
                const collision = this.collisions[guid];
                const entity = collision.entity;
                const entityCollision = entity.collision;
                const entityRigidbody = entity.rigidbody;
                const others = collision.others;
                const length = others.length;
                let i = length;
                while (i--) {
                    const other = others[i];
                    // if the contact does not exist in the current frame collisions then fire event
                    if (!frameCollision || frameCollision.others.indexOf(other) < 0) {
                        // remove from others list
                        others.splice(i, 1);

                        if (entity.trigger) {
                            // handle a trigger entity
                            if (entityCollision) {
                                entityCollision.fire('triggerleave', other);
                            }
                            if (other.rigidbody) {
                                other.rigidbody.fire('triggerleave', entity);
                            }
                        } else if (!other.trigger) {
                            // suppress events if the other entity is a trigger
                            if (entityRigidbody) {
                                entityRigidbody.fire('collisionend', other);
                            }
                            if (entityCollision) {
                                entityCollision.fire('collisionend', other);
                            }
                        }
                    }
                }

                if (others.length === 0) {
                    delete this.collisions[guid];
                }
            }
        }
    }
}

export { PhysicsProvider };
