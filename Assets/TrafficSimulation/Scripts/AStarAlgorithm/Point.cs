using Assets.Scripts;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TrafficSimulation;
using UnityEngine;
using static TMPro.SpriteAssetUtilities.TexturePacker_JsonArray;

namespace Assets.Scripts
{
    public class Point
    {
        public NodeInfo goal;
        public NodeInfo actual;

        public Point parent;
        public int previousSeg; //0: waypoint 0

        public Vector2 ProducedBy;

        public float g;
        public float f;

        public Point(NodeInfo actual, NodeInfo goal)
        {
            this.actual = actual;
            this.goal = goal;
        }

        public float funHeuristica()
        {
            float aux = Vector3.Distance(goal.transf.position, actual.transf.position);
            f = aux + g;
            return f;
        }

        public void caminoInverso()
        {
            if(actual.waypoint > 0)
            {
                int aux = actual.waypoint;
                ProducedBy = new Vector2(actual.segment, aux--);
            }
            else
            {
                ProducedBy = new Vector2(previousSeg, 0);
            }

        }

    }
}
