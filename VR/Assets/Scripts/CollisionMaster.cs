using System.Collections.Generic;
using UnityEngine;

public class CollisionMaster : MonoBehaviour
{
    static GameObject[] allObjects;
    static List<GameObject> ColliderObjects = new List<GameObject>();
    static int numObjects;
    // Start is called before the first frame update
    void Start()
    {
        allObjects = FindObjectsOfType<GameObject>();
        numObjects = allObjects.Length;
        foreach (GameObject obj in allObjects)
        {
            if (obj.GetComponent<Collision>() != null)
            {
                ColliderObjects.Add(obj);
            }
        }

    }

    // Update is called once per frame
    void Update()
    {
    }
    private void FixedUpdate()
    {
        //BROAD PHASE: 
        //Sort and sweep algorithm on x-axis: 
        HashSet<(string, string)> intersectedObjects = BroadPhaseClass.FindIntersections(ref ColliderObjects);
        // Debug.Log(intersectedObjects.Count);
        //NARROW PHASE
        NarrowPhaseClass.NarrowPhase(ref intersectedObjects);
    }

}
