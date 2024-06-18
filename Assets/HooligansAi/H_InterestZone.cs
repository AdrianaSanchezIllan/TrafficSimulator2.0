using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class H_InterestZone : MonoBehaviour
{
    public List<H_InterestLocation> locations = new List<H_InterestLocation>();
    public int numFreePos;//static?
    public string interestAction = "Stadium"; // Acci�n que se realiza en esta zona
    public GameObject actionIndicatorPrefab; // Prefab para mostrar la acci�n actual

    private void Start()
    {
        locations.Clear();
        foreach (Transform child in transform)
        {
            H_InterestLocation location = child.GetComponent<H_InterestLocation>();
            if (location != null)
            {
                locations.Add(location);
            }
            else
            {
                Debug.LogWarning("El hijo " + child.name + " no tiene el componente H_InterestLocation.");
            }
        }
        numFreePos = locations.Count;
    }
    public int FreePosRemaining(){
        return numFreePos;
    }

    private void OnTriggerEnter(Collider other)
    {
        HoolBehaviour hoolBehaviour = other.GetComponent<HoolBehaviour>();
        if (hoolBehaviour != null)
        {
            Debug.Log("Entered interest zone: " + interestAction);
            hoolBehaviour.currentZone = this;
            hoolBehaviour.currentAction = this.interestAction;
            
            hoolBehaviour.agent.isStopped= true;
            
            Debug.Log("6-------> ONTRIGGERENTER");
            //Debug.Log("se ha quedado quietesito");
        }
    }

    private void OnTriggerExit(Collider other)
    {
        HoolBehaviour hoolBehaviour = other.GetComponent<HoolBehaviour>();
        if (hoolBehaviour != null)
        {
            hoolBehaviour.currentZone = null;
            Debug.Log("Exited interest zone");
        }
    }

    // M�todo para obtener la primera localizaci�n libre
    public H_InterestLocation GetFirstFreeLocation()
    {
        foreach (var location in locations)
        {
            
            if (!location.IsOccupied)
            {
                Debug.Log("10-------> GETFIRSTFREELOCATION");
                location.Occupy();
                //numFreePos--;
                return location;
            }
        }
        return null; // Si no hay localizaciones libres, devuelve null
    }

    // M�todo para liberar una localizaci�n
    public void VacateLocation(H_InterestLocation location)
    {
        if (locations.Contains(location))
        {
            location.Vacate();
            numFreePos++;
        }
    }

    // M�todo para obtener la localizaci�n libre m�s cercana
    public H_InterestLocation GetClosestLocation(Transform pedestrianTransform)
    {
        H_InterestLocation closestLocation = null;
        float closestDistance = float.MaxValue;

        foreach (var location in locations)
        {
            float distance = Vector3.Distance(pedestrianTransform.position, location.transform.position);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestLocation = location;
            }
        }


        return closestLocation;
    }

    // M�todo para obtener la localizaci�n libre m�s lejana
    public H_InterestLocation GetFurthestLocation(Transform pedestrianTransform)
    {
        H_InterestLocation furthestLocation = null;
        float furthestDistance = float.MinValue;

        foreach (var location in locations)
        {
            float distance = Vector3.Distance(pedestrianTransform.position, location.transform.position);
            if (distance > furthestDistance)
            {
                furthestDistance = distance;
                furthestLocation = location;
            }
        }

        return furthestLocation;
    }
}
