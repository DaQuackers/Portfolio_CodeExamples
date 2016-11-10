using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.DirectX;
using FullSailAFI.SteeringBehaviors.Core;

namespace FullSailAFI.SteeringBehaviors.StudentAI
{
    public class Flock
    {
        public float AlignmentStrength { get; set; }
        public float CohesionStrength { get; set; }
        public float SeparationStrength { get; set; }
        public List<MovingObject> Boids { get; protected set; }
        public Vector3 AveragePosition { get; set; }
        protected Vector3 AverageForward { get; set; }
        public float FlockRadius { get; set; }

        #region Constructors
        public Flock()
        {
        }
        #endregion

        #region Helper Methods

        private void CalculateAverages()
        {
            AveragePosition = Vector3.Empty;
            AverageForward = Vector3.Empty;
            foreach(MovingObject boid in Boids)
            {
                AveragePosition += boid.Position;
                AverageForward += boid.Velocity;
            }
            float divide = 1.0f / Boids.Count;
            AverageForward = AverageForward*divide;
            AveragePosition = AveragePosition*divide;
        }

        private Vector3 CalculateAlignmentAcceleration(MovingObject boid)
        {
            Vector3 ret; ret = AverageForward * (1.0f/boid.MaxSpeed);
            double dist = ret.Length();
            if (dist > 1)
                ret.Normalize();
            return ret*AlignmentStrength;
        }

        private Vector3 CalculateCohesionAcceleration(MovingObject boid)
        {
            Vector3 ret = AveragePosition - boid.Position;
            double dist = ret.Length();
            ret.Normalize();
            if (dist < FlockRadius)
                ret = ret * ((float)dist / FlockRadius);
            return ret * CohesionStrength;
        }

        private Vector3 CalculateSeparationAcceleration(MovingObject boid)
        {
            Vector3 ret = Vector3.Empty;
            foreach(MovingObject notme in Boids)
            {
                if(notme.Position != boid.Position)
                {
                    Vector3 vb = boid.Position - notme.Position;
                    float dist = vb.Length();
                    float safedist = boid.SafeRadius + notme.SafeRadius;
                    if(dist < safedist)
                    {
                        vb.Normalize();
                        vb = vb * ((safedist - dist) / safedist);
                        ret += vb;
                    }
                }
            }
            if (ret.Length() > 1.0f)
                ret.Normalize();
            return ret * SeparationStrength;
        }

        #endregion

        #region TODO

        public virtual void Update(float deltaTime)
        {
            CalculateAverages();
            foreach(MovingObject boid in Boids)
            {
                Vector3 accel = CalculateAlignmentAcceleration(boid);
                accel += CalculateCohesionAcceleration(boid);
                accel += CalculateSeparationAcceleration(boid);
                accel = accel*(boid.MaxSpeed * deltaTime);
                boid.Velocity += accel;
                if(boid.Velocity.Length() > boid.MaxSpeed)
                {
                    boid.Velocity.Normalize();
                    boid.Velocity = boid.Velocity* boid.MaxSpeed;
                }
                boid.Update(deltaTime);
            }
            return;
        }
        #endregion
    }
}
