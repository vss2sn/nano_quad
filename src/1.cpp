void update2()
{
  ros::Time time = ros::Time::now();
  double dt = time.toSec() - previousTime.toSec();

// PID controller with distance from goal as inout; tuned to output goal velocity
// Output is simply p+i+d, not prev distance+p+i+d
  for (c=0;c<6;c++)
  {
    error[c] = gp[c] - cp[c];
    integral[c] += error[c] * dt;
    integral[c] = std::max(std::min(integral[c], integratorMax), integratorMin);
    double p = kp[c] * error[c];
    double d = 0;
    if (dt > 0)
    {
        d = kd[c] * (error[c] - previousError[c]) / dt;
        cv[c] = (error[c]-previousError[c]) / dt;
    }
    double i = ki[c] * integral[c];
    //gv[c] = (p + d + i);
    gv[c] = p;
    previouserror[c] = error[c];
    gv[c]=std::max(minOutput,(std::min(gv[c],maxOutput)));

// Using P input of position as input for velocity
// Output is simply p+i+d, not prev velocity+p+i+d

    error_v[c] = gv[c] - cv[c];
    integral_v[c] += error_v[c] * dt;
    integral_v[c] = std::max(std::min(integral_v[c], integratorMax), integratorMin);
    p = kp_v[c] * error_v[c];
    if (dt > 0)
    {
        d = kd[c] * (error_v[c] - previousError_v[c]) / dt;
    }
    i = ki_v[c] * integral_v[c];
    output[c] = (p + d + i);
    previouserror_v[c] = error_v[c];
    output[c] = std::max(minOutput,(std::min(output[c],maxOutput)));
    previous_output[c] = output[c];
  }
  previousTime = time;

}
