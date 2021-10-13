using UnityEngine;

public class TargetScript : MonoBehaviour
{
    private const float Speed = 2;
    public Vector3 _target;

    private void Start() => 
        _target = transform.position;

    private void Update()
    {
        if (!OnTarget())
        {
            var distance = Time.deltaTime * Speed;
            var direction = _target - transform.position;
            direction.Normalize();
            transform.position += direction*distance;
        }
    }

    private void ResetTarget() => 
        _target = Random.onUnitSphere * 5;

    private bool OnTarget() => 
        Vector3.Distance(transform.position, _target) < 1e-1;

    private void  OnTriggerEnter (Collider targetObj) 
    {
        if(targetObj.gameObject.CompareTag("Actor"))
            ResetTarget();
    }
}
