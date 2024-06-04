using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;

public class PedestrianBehaviour : MonoBehaviour
{
    public NavMeshAgent agent;
    public float range; // Radio de deambulación
    public Transform centrePoint; // Centro del área de deambulación

    private Animator animator;
    private static readonly int IsWalking = Animator.StringToHash("isWalking");

    private GameObject currentActionIndicator;
    public InterestZone currentZone;
    private Transform currentLocation;

    Transform location;
    private bool isActionCompleted = false;
    private bool inTrafficLight = false;
    private bool isCrossingStreet = false;
    private TrafficLights trafficLight;

    private void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        if (animator == null)
        {
            Debug.LogError("Animator not found!");
        }

        if (agent == null)
        {
            agent = gameObject.AddComponent<NavMeshAgent>();
        }

        if (centrePoint == null)
        {
            centrePoint = transform;
        }
    }

    private void OnTriggerEnter(Collider collider)
    {
        if (collider.gameObject.TryGetComponent<TrafficLights>(out TrafficLights semaforo))
        {
            trafficLight = semaforo;
            if (!semaforo.IsRedForCars() || semaforo.TimeToChange() < 5)
            {
            inTrafficLight = true;
        }
    }
        inTrafficLight = false;
    }
    private void OnTriggerExit(Collider other)
    {
        trafficLight = null;
        inTrafficLight = false;
    }

    private bool InTrafficLight()
    {
        return inTrafficLight;
    }

    #region FSM States

    // Estado de deambulación
    public void StartRoaming()
    {
        StartCoroutine(Roam());
    }

    private IEnumerator Roam()
    {
        while (true)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                Vector3 point;
                if (RandomPoint(centrePoint.position, range, out point))
                {
                    Debug.DrawRay(point, Vector3.up, Color.blue, 1.0f);
                    agent.SetDestination(point);
                }
            }

            bool isMoving = agent.velocity.magnitude > 0.1f;
            animator.SetBool(IsWalking, isMoving);

            if (inTrafficLight && !trafficLight.IsRedForCars())
            {
                agent.isStopped = true;
                StartCrossingStreet();
                yield break;
            }

            yield return null;
        }
    }
    
    // Estado de cruce de calle
    public void StartCrossingStreet()
    {
        StartCoroutine(WaitForGreenLight());
    }

    private IEnumerator WaitForGreenLight()
    {
        while (!trafficLight.IsRedForCars())
        {
            yield return new WaitForSeconds(0.5f); // Esperar 0.5 segundos antes de verificar nuevamente
        }

        isCrossingStreet = true;
        StartRoaming();
    }

    #endregion

    public void MoveTo(Vector3 destination)
    {
        agent.SetDestination(destination);
        StartCoroutine(CheckIfArrived());
    }

    private IEnumerator CheckIfArrived()
    {
        while (true)
        {
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                agent.isStopped = true;
                yield break; // Terminar coroutine cuando el agente llega al destino
            }

            bool isMoving = agent.velocity.magnitude > 0.1f;
            animator.SetBool(IsWalking, isMoving);

            yield return null;
        }
    }

    public void PerformAction(string action)
    {
        switch (action)
        {
            case "Coffee":
                Debug.Log("Tomando un café.");
                break;
            case "Park":
                Debug.Log("Dando una vuelta corriendo.");
                break;
            case "Supermarket":
                Debug.Log("Comprando víveres.");
                break;
            default:
                Debug.LogWarning("Acción desconocida: " + action);
                break;
        }
    }

    public Status EnterInterestZone()
    {
        currentLocation = currentZone.GetAvailableLocation();
        Debug.Log("current location" + currentLocation);
        return currentLocation != null ? Status.Success : Status.Failure;
    }

    public Status MoveToLocation()
    {
        if (currentLocation != null)
        {
            Debug.Log(currentLocation);
            MoveTo(currentLocation.position);
            if (isPathComplete())
            {
                Debug.Log("he llegado a la silla");
                return Status.Success;
            }
            return Status.Running;
        }
        return Status.Failure;
       
    }

    public Status PerformActionBT()
    {
        if (currentZone != null)
        {
            StartCoroutine(PerformInterestAction());
            return Status.Running;
        }
        return Status.Failure;
    }

    private IEnumerator PerformInterestAction()
    {
        while (agent.remainingDistance > agent.stoppingDistance)
        {
            yield return null;
        }
        PerformAction(currentZone.interestAction);
        agent.isStopped = true;

        if (currentActionIndicator != null)
        {
            Destroy(currentActionIndicator);
        }

        if (currentZone.actionIndicatorPrefab != null)
        {
            currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
            // Ajustar la posición de la imagen por encima del personaje
            currentActionIndicator.transform.localPosition = new Vector3(0, 2.0f, 0);
        }

        // Simular duración de la acción
        yield return new WaitForSeconds(10.0f); // Por ejemplo,10 segundos para tomar café

        if (currentActionIndicator != null)
        {
            Destroy(currentActionIndicator);
        }

        agent.isStopped = false;
        currentZone.VacateLocation(currentLocation); // Liberar la ubicación
        currentLocation = null;
        isActionCompleted = true;
        StartRoaming(); // Retomar el roaming después de completar la acción
    }

    private bool RandomPoint(Vector3 center, float range, out Vector3 result)
    {
        Vector3 randomPoint = center + Random.insideUnitSphere * range;
        NavMeshHit hit;
        if (NavMesh.SamplePosition(randomPoint, out hit, 1.0f, NavMesh.AllAreas))
        {
            result = hit.position;
            return true;
        }

        result = Vector3.zero;
        return false;
    }

    #region Behaviour Tree Nodes

    public Status CheckIfCanPerformAction()
    {
        return currentZone != null ? Status.Success : Status.Failure;
    }

    public Status CheckIfCanCrossStreet()
    {
        return trafficLight != null && trafficLight.IsRedForCars() ? Status.Success : Status.Failure;
    }

    public Status IsInCoffeeShop()
    {
        if (currentZone != null && currentZone.interestAction == "Coffee")
        {
            return Status.Success;
        }
        return Status.Failure;
    }

    public Status IsInPark()
    {
        if (currentZone != null && currentZone.interestAction == "Park")
        {
            return Status.Success;
        }
        return Status.Failure;
    }

    public Status IsInSupermarket()
    {
        if (currentZone != null && currentZone.interestAction == "Supermarket")
        {
            return Status.Success;
        }
        return Status.Failure;
    }

    #endregion

    public bool IsInterestZone()
    {
        return currentZone != null;
    }

    public bool HasCompletedAction()
    {
        return isActionCompleted;
    }


    public bool isPathComplete()
    {
        return (!agent.pathPending &&
                agent.remainingDistance <= agent.stoppingDistance &&
                (!agent.hasPath || agent.velocity.sqrMagnitude == 0f));
    }

    public Status PathStatus()
    {
        if (!agent.pathPending &&
            agent.remainingDistance <= agent.stoppingDistance &&
            (!agent.hasPath || agent.velocity.sqrMagnitude == 0f))
        {
            return Status.Success;
        }
        return Status.Running;
    }

    public Status ActionHasBeenCompleted()
    {
        if(isActionCompleted)
        {
            return Status.Success;
        }
        return Status.Running;
    }
}
