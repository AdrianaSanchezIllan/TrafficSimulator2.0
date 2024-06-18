using System;
using System.Collections.Generic;
using TrafficSimulation;
using UnityEngine;
using Object = UnityEngine.Object;

namespace Assets.Scripts
{
    public class NodeInfo : ICloneable
    {
        public int segment { get; set; }
        public int waypoint { get; set; }
        public Transform transf { get; set; }

        public float WalkCost { get; private set; }

        public NodeInfo(int segment, int waypoint, Transform transform)
        {
            this.WalkCost = 1.0f;
            this.segment = segment; 
            this.waypoint = waypoint;
            this.transf = transform;
        }

        public List<NodeInfo> Neightbors(TrafficSystem trafficSystem)
        {
            List<NodeInfo> options = new List<NodeInfo>();
            int aux = waypoint;
            int nextWP = aux+1;
            int nextSeg = segment;

            if (nextWP >= trafficSystem.segments[segment].waypoints.Count)
            {
                nextWP = 0;
                if (trafficSystem.segments[segment].nextSegments.Count == 0)
                {
                    nextSeg = 0;
                }
                else
                {
                    foreach (var segment in trafficSystem.segments[segment].nextSegments)
                    {
                        options.Add(new NodeInfo(segment.id, nextWP, trafficSystem.segments[segment.id].waypoints[nextWP].transform));
                    }
                    return options; //devuelve lista de vecinos
                }
            }
            options.Add(new NodeInfo(nextSeg, nextWP, trafficSystem.segments[nextSeg].waypoints[nextWP].transform)); //devuelve lista con unico vecino posible
            return options;
            
        }

        
        public object Clone()
        {
            var result = new NodeInfo(this.segment, this.waypoint, this.transf)
            {
                WalkCost = this.WalkCost
            };

            return result;
        }
    }
}