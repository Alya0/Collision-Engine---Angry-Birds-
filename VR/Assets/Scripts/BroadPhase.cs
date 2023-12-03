using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BroadPhaseClass
{

    public static int axis = 0;
    public static void BroadPhase()
    {

    }

    public static HashSet<(string, string)> FindIntersections(ref List<GameObject> ColliderObjects)
    {

        Vector3 centerSum = new Vector3(0, 0, 0);
        Vector3 centerSqSum = new Vector3(0, 0, 0);
        List<(float, string)> endpoints = new List<(float, string)>();
        foreach (GameObject obj in ColliderObjects)
        {
            //to calculate variance: 
            Vector3 center = obj.transform.position;
            centerSum += center;
            centerSqSum += new Vector3(center.x * center.x, center.y * center.y, center.z * center.z);

            Collision collision = obj.GetComponent<Collision>();
            endpoints.Add((collision.start[axis], obj.name));
            endpoints.Add((collision.end[axis], obj.name));
        }
        endpoints.Sort((p1, p2) => p1.Item1.CompareTo(p2.Item1));

        //to calculate variance
        centerSum /= ColliderObjects.Count;
        centerSqSum /= ColliderObjects.Count;
        Vector3 variance = centerSqSum - (new Vector3(centerSum.x * centerSum.x, centerSum.y * centerSum.y, centerSum.z * centerSum.z));
        float maxVariance = Mathf.Max(Mathf.Max(variance.x, variance.y), variance.z);
        if (maxVariance == variance.x) axis = 0;
        else if (maxVariance == variance.y) axis = 1;
        else axis = 2;
        //end of calculating variance and changing axis;

        HashSet<(string, string)> intersections = new HashSet<(string, string)>();
        HashSet<string> active = new HashSet<string>();
        foreach ((float, string) endpoint in endpoints)
        {
            if (active.Contains(endpoint.Item2))
            {
                active.Remove(endpoint.Item2);
            }
            else
            {
                foreach (string other in active)
                {
                    intersections.Add((endpoint.Item2, other));
                }
                active.Add(endpoint.Item2);
            }

        }
        return intersections;
    }

}