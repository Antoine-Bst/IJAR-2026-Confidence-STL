/* ============================================================================
 * D Y N I B E X - Definition of the Simulation
 * ============================================================================
 * Copyright   : ENSTA ParisTech
 * License     : This program can be distributed under the terms of the GNU LGPL.
 *               See the file COPYING.LESSER.
 *
 * Author(s)   : Julien Alexandre dit Sandretto and Alexandre Chapoutot
 * Created     : Jul 18, 2014
 * Modified    : Oct, 2015 (addition of DAE)
 * Author(s) of modification   : Julien Alexandre dit Sandretto and Olivier Mullier
 * Modified    : Jul 25, 2018
 * Sponsored   : This research benefited from the support of the "Chair Complex Systems Engineering - Ecole Polytechnique,
 * THALES, DGA, FX, DASSAULT AVIATION, DCNS Research, ENSTA ParisTech, Telecom ParisTech, Fondation ParisTech and FDO ENSTA"
 * ---------------------------------------------------------------------------- */
#ifndef IBEX_SIMU_H
#define IBEX_SIMU_H

#include <fstream>
#include "ibex_integrate.h"
#include <stdlib.h>

// #ifndef __PRINT_DEBUG__
// #define __PRINT_DEBUG__
// #endif

namespace ibex{
  //namespace std{

  enum Method {IEULER,IMIDPOINT,RADAU3,HEUN,TAYLOR4,LA3,LC3,RK4,RADAU3_DAE, RADAU5, GL4, GL6, KUTTA3};

  class simulation
  {
  public:
    double time_T;
    int process;
    double atol;
    int test;
    Method meth;
    int nb_var;
    bool failed; //if simulation failed

    bool monotony_active;
    
    //alarm
    CtcFixPoint* alarm_ctc=NULL;

    //ode
    std::list<solution_j> list_solution_j;
	ivp_ode* embedded_ode;

	//for bwd
	ivp_ode* inverse_ode;

    //dae
    std::list<solution_j_dae> list_solution_j_dae;
    ivp_dae_h1* embedded_dae=NULL;

    //discrete
    ivp_discrete* embedded_dis=NULL;
    double hdis=1.0;
    
    //test
    std::list<solution_g> list_solution_g;

    //add june 2020: list of picard under zonotopes form
    std::list<Affine2Vector> zonotopicard;
    std::list<Interval> zonotopicard_time;


    //remember the last solution in affine form
    Affine2Vector* last_aff;


    //set alarm
    void set_alarm(const Array<NumConstraint>& csp)
    {
      Array<NumConstraint> csp_temp(csp);
      CtcHC4* hc4 = new CtcHC4(csp_temp,tol_ctc);
      Ctc3BCid* cid = new Ctc3BCid(*hc4);
      alarm_ctc = new CtcFixPoint(*cid,tol_ctc);
    }

    //var for monotony use
    void active_monotony()
    {
      monotony_active=true;
    }

    void inactive_monotony()
    {
      monotony_active=false;
    }

    // Getter and Setter for the hmin and hmax parameters
    static double getHmin () {
      return dynibex_hmin;
    }

    static void setHmin (double _hmin) {
      assert(_hmin > 0.0);
      dynibex_hmin = _hmin;
    }

    static double getHmax () {
      return dynibex_hmax;
    }

    static void setHmax (double _hmax) {
      assert(_hmax > 0.0);
      dynibex_hmax = _hmax;
    }
    //==========MODIF PERSO===========
        IntervalVector eval_vector_j(const IntervalVector& box_j) const
    {
      assert(embedded_ode != nullptr);
      assert(embedded_ode->ydot != nullptr);

      return embedded_ode->ydot->eval_vector(box_j);
    }
    //=========Evaluation de Picard=========
    
    //fill genericfrom whatever solution (ode or dae)
    void sol_to_generic(std::list<solution_j> list_sol)
    {
      std::list<solution_j>::iterator iterator_list;
      for(iterator_list=list_sol.begin();iterator_list!=list_sol.end();iterator_list++)
	{
	  solution_g sol_temp(iterator_list->time_j, iterator_list->box_j1, iterator_list->box_jn, iterator_list->box_jnh,
			      //iterator_list->box_jn_aff, iterator_list->box_jnh_aff, // Add affine forms
			      nb_var); // Add affine solution !
	  list_solution_g.push_back(sol_temp);
	  iterator_list->destructor();
	}
    }

    void sol_to_generic(std::list<solution_j_dae> list_sol)
    {
      std::list<solution_j_dae>::iterator iterator_list;
      for(iterator_list=list_sol.begin();iterator_list!=list_sol.end();iterator_list++)
	{
	  solution_g sol_temp(iterator_list->time_j, iterator_list->box_j1, iterator_list->box_jn, iterator_list->box_jnh,
			      //iterator_list->box_jn_aff, iterator_list->box_jnh_aff, // Add affine forms
			      nb_var);
	  list_solution_g.push_back(sol_temp);
	  iterator_list->destructor();
	}
    }
    
    //compute tube for discrete systems and fill generic
    void compute_discrete()
    {
    	double tn=embedded_dis->t0;
    	Affine2Vector jn_aff = Affine2Vector(*embedded_dis->yinit_aff);
    	while (tn<time_T)
    	{
    		Interval time(tn,tn+hdis);
    		IntervalVector* jn=new IntervalVector(jn_aff.itv());
    		jn_aff = embedded_dis->ydot->eval_affine2_vector(jn_aff);
    		IntervalVector* jnh= new IntervalVector(jn_aff.itv());
    		IntervalVector* j1= new IntervalVector((*jn)|(*jnh));
    		solution_g sol_temp(time, j1, jn, jnh,nb_var);
		list_solution_g.push_back(sol_temp);
    		tn+=hdis;
    	}
    
    }
    
    
    
    /** compute solutions after solutions till t >= time_T*/
    int run_simulation(bool with_zonotube=true){
    ///remettre zonotube à false après
      double tn ;
      solution_j sol_temp;
      solution_j_dae sol_temp_dae;
      double fac;
      double newh;
      
      if (embedded_dis!=NULL)
      {
      	compute_discrete();
      	return 1;
      
      }
      bool simu_start=false; //to avoid first step
      
      do
	{
	  //reuse of the first (and verified step) to build the followers
	  if (!list_solution_j.empty())
	    {
	      sol_temp = list_solution_j.back();
	      //initialize next solution
	      tn = sol_temp.time_j.ub();
	      fac = sol_temp.factor;
	      newh = std::abs(sol_temp.time_j.diam())*fac; //we apply the factor estimated the last step
	    }
	  else
	    {
	      sol_temp_dae = list_solution_j_dae.back();
	      //initialize next solution
	      tn = sol_temp_dae.time_j.ub();
	      fac = sol_temp_dae.factor;
	      newh = std::abs(sol_temp_dae.time_j.diam())*fac; //we apply the factor estimated the last step
	    }

	  //alarm verification
	  if ((alarm_ctc!=NULL) && (simu_start))
	  {
	  	IntervalVector temp=IntervalVector(*sol_temp.box_j1);
	  	alarm_ctc->contract(temp);
	  	if (!temp.is_empty())
	  		std::cerr << "Alarm activated at t=" << sol_temp.time_j << std::endl; 
	  }
	  simu_start=true;


	//add june 2020: zono tube
	if (with_zonotube)
	{
		//quick computation of picard under zono form with euler
		Affine2Vector temp = *sol_temp.box_jn_aff + Interval(0,sol_temp.time_j.diam())*embedded_ode->ydot->eval_vector(*sol_temp.box_j1);
		zonotopicard.push_back(temp);
		zonotopicard_time.push_back(sol_temp.time_j);
                //zonotopicard.push_back(*sol_temp.box_jnh_aff); //TODO ajouter picard et pas jnh
	}



	  if (tn < time_T)
	    {
	      double delta_t = time_T-tn;

	      if (delta_t < newh) newh = delta_t;

	      switch(meth)
		{
		case IEULER:
		  {
		    solution_j_ieuler u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case IMIDPOINT:
		  {
		    solution_j_imidpoint u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case RADAU3:
		  {
		    solution_j_radau3 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case HEUN:
		  {
		    solution_j_heun u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    //sol_temp.flush(); // used to scrunch affine form into center + radius
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case TAYLOR4:
		  {
		    solution_j_tayl4 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case KUTTA3:
		  {
		    solution_j_kutta3 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case LA3:
		  {
		    solution_j_la3 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case LC3:
		  {
		    solution_j_lc3 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case RK4:
		  {
		    solution_j_rk4 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case GL4:
		  {
		    solution_j_gauss4 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case GL6:
		  {
		    solution_j_gauss6 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }
		case RADAU3_DAE:
		  {
		    solution_j_dae u_j(*sol_temp_dae.box_jnh_aff,*sol_temp_dae.box_xnh_aff, tn,newh, embedded_dae,atol,fac);
		    u_j.compute_oneStep(*sol_temp_dae.box_jnh_aff,*sol_temp_dae.box_xnh_aff,embedded_dae);
		    sol_temp_dae.flush();
 		    list_solution_j_dae.push_back(u_j);
		    //cout << "\nSolution at t=" << sol_temp_dae.time_j.ub() << " : " << sol_temp_dae.box_jnh_aff->itv() << endl;
		    break;

		  }
		case RADAU5:
		  {
		    solution_j_radau5 u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		    u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode,monotony_active);
		    sol_temp.flush();
		    list_solution_j.push_back(u_j);
		    break;
		  }

		default :
		  {
		    /*solution_j u_j(*sol_temp.box_jnh_aff,tn,newh, embedded_ode,atol,fac);
		      u_j.compute_oneStep(*sol_temp.box_jnh_aff,embedded_ode);
		      sol_temp.flush();
		      list_solution_j.push_back(u_j);*/
		    std::cout << "This method does not exist !" << std::endl;
		    tn = time_T;
		  }

		}
	    }
	  if (!list_solution_j.empty())
	    tn = sol_temp.time_j.ub();
	  else
	    tn = sol_temp_dae.time_j.ub();

	}while (tn < time_T);
      ///////////////**************//////////////
      if (tn > 1e7)
	{
	  failed = true;
	  tn = time_T;
	}
      ///////////////**************//////////////
      else   /////
	{
	  failed=false;
	}


      if (!list_solution_j.empty())
	{
	  //print of the last solution
	  solution_j sol_temp = list_solution_j.back();
	  //sol_temp.print_soljn();
	  // 		std::cout << "\nSolution at t=" << tn << " : " << sol_temp.box_jnh_aff->itv() << std::endl;
	  //cout << "Diam : " << sol_temp.box_jnh->diam() << endl;

	  LOGGER->Log("Solution at t=%f : ",tn);
	  LOGGER->Log_sol(sol_temp.box_jnh_aff->itv());
	  LOGGER->Log_end();

	  if (!failed)
	    last_aff = new Affine2Vector(*sol_temp.box_jnh_aff);
	  else
	    last_aff = new Affine2Vector(*sol_temp.box_jnh,true);


	//add june 2020: zono tube
	if (with_zonotube)
	{
		//zonotopicard.push_back(*last_aff); //TODO ajouter picard et pas jnh
		//quick computation of picard under zono form with euler
		Affine2Vector temp = *sol_temp.box_jn_aff + Interval(0,sol_temp.time_j.diam())*embedded_ode->ydot->eval_vector(*sol_temp.box_j1);
		zonotopicard.push_back(temp);
		zonotopicard_time.push_back(sol_temp.time_j);
	}

	  sol_to_generic(list_solution_j);
	  list_solution_j.clear();

	  //build of inverse ode
	  inverse_ode = new ivp_ode(embedded_ode, true, tn, *sol_temp.box_jnh_aff);
	  inverse_ode->frechet_precomputing(order_of_method(meth));
	}
      else
	{
	  //print of the last solution
	  solution_j_dae sol_temp_dae = list_solution_j_dae.back();
	  std::cout << "\nSolution at t=" << tn << " : " << sol_temp_dae.box_jnh_aff->itv() << std::endl;

	  LOGGER->Log("Solution at t=%f : ",tn);
	  LOGGER->Log_sol(sol_temp_dae.box_jnh_aff->itv());
	  LOGGER->Log_end();

	  if (!failed)
	    last_aff = new Affine2Vector(*sol_temp_dae.box_jnh_aff);
	  else
	    last_aff = new Affine2Vector(*sol_temp_dae.box_jnh,true);

	  sol_to_generic(list_solution_j_dae);
	  list_solution_j_dae.clear();

	}


      return 1;
    };
    /**find the box of a stack which contains a sub-box (stack size nbvar/2, y size nbvar)*/
    IntervalVector find_box(std::list<IntervalVector>* stack, IntervalVector y)
    {

      std::list<IntervalVector>::iterator iterator_list;

      IntervalVector suby = y.subvector(0,y.size()/2 - 1);
      for(iterator_list=stack->begin();iterator_list!=stack->end();iterator_list++)
	{
	  IntervalVector yv =*iterator_list;


	  if (!(suby & yv).is_empty())
	    return yv;
	}

      return IntervalVector(y.size()/2);
    }

    //return order of derivatives (order method +1)
    int order_of_method(Method _meth)
    {
      switch(meth)
	{
	case IEULER:
	  {
	    return 2;
	    break;
	  }
	case IMIDPOINT:
	  {
	    return 3;
	    break;
	  }
	case RADAU3:
	  {
	    return 4;
	    break;
	  }
	case HEUN:
	  {
	    return 3;
	    break;
	  }
	case TAYLOR4:
	  {
	    return 5;
	    break;
	  }
	case LA3:
	  {
	    return 5;
	    break;
	  }
	case LC3:
	  {
	    return 5;
	    break;
	  }
	case RK4:
	  {
	    return 5;
	    break;
	  }
	case RADAU3_DAE:
	  {
	    return 4;
	    break;
	  }
	case RADAU5:
	  {
	    return 6;
	    break;
	  }
	case GL4:
	  {
	    return 5;
	    break;
	  }
	case GL6:
	  {
	    return 7;
	    break;
	  }
	case KUTTA3:
	  {
	    return 4;
	    break;
	  }
	default :
	  {
	    return 5;
	  }
	}
      return 6; //max of order method +1
    }

    double pred_h(ivp_ode* _ode, double atol)
    {
      IntervalMatrix J=_ode->eval_jacobian_init();//ode->ydot->jacobian(_ode->yinit);
      double max_J = 0;
      for (int i=0; i < J.nb_rows(); i++)
	{
	  for (int j=0; j < J.nb_cols(); j++)
	    {
	      if (abs(J[i][j]).ub() > max_J)
		max_J = abs(J[i][j]).ub();
	    }

	}
      return max_J;
    }

    /**constructor*/
    simulation(ivp_ode* _ode, double T,Method _meth = RK4, double a = 1e-6){
      process = 0;
      embedded_ode = _ode;
      time_T = T;
      atol = a;
      meth = _meth;
      nb_var=_ode->nbvar;
      monotony_active = false;
      double coef = pred_h(_ode, atol);
      if (coef == 0.0)
	coef=1.0;
      double h = std::min(std::min(dynibex_hmax,0.01/coef),T/2.0);
      if (embedded_ode->get_lte_computation() == SYMBOLIC){
	embedded_ode->frechet_precomputing(order_of_method(meth));
      } else {
	embedded_ode->dag_precomputing(order_of_method(meth));
      }

      solution_j u_j0(*embedded_ode->yinit_aff,embedded_ode->t0,h,atol);
      list_solution_j.push_back(u_j0);
    }

    /**constructor*/
    simulation(ivp_ode* _ode, double T,Method _meth, double a, double _h){
      process = 0;
      embedded_ode = _ode;
      time_T = T;
      atol = a;
      meth = _meth;
      nb_var=_ode->nbvar;
      monotony_active = false;

      double h = _h;
      if (embedded_ode->get_lte_computation() == SYMBOLIC){
	embedded_ode->frechet_precomputing(order_of_method(meth));
      } else {
	embedded_ode->dag_precomputing(order_of_method(meth));
      }

      solution_j u_j0(*embedded_ode->yinit_aff,embedded_ode->t0,h,atol);
      list_solution_j.push_back(u_j0);
    }
    
    //for discrete systems, _h is the duration of a step and T the duration of the simulation
    simulation(ivp_discrete* _dis, double T, double _h){
      process = 0;
      embedded_dis = _dis;
      time_T = T;

      nb_var=_dis->nbvar;
      monotony_active = false;

      hdis = _h;
    }

    // 	//constructor
    // 	simulation(ivp_ode* _ode, double T){
    // 	  process = 0;
    // 	  embedded_ode = _ode;
    // 	  time_T = T;
    // 	  atol = 1e-6;
    // 	  meth = RK4;
    // 	  nb_var=_ode->nbvar;
    // 	  monotony_active = false;
    // 	  double coef = pred_h(_ode, atol);
    // 	  if (coef == 0.0)
    // 	    coef=1.0;
    // 	  double h = std::min(std::min(dynibex_hmax,0.01/coef),T/2.0);
    //
    // 	  embedded_ode->frechet_precomputing(order_of_method(meth));
    // 	  solution_j u_j0(*embedded_ode->yinit_aff,embedded_ode->t0,h,atol);
    // 	  list_solution_j.push_back(u_j0);
    // 	}
    //
    // 	//constructor
    // 	simulation(ivp_ode* _ode, double T,Method _meth){
    // 	  process = 0;
    // 	  embedded_ode = _ode;
    // 	  time_T = T;
    // 	  atol = 1e-6;
    // 	  meth = _meth;
    // 	  nb_var=_ode->nbvar;
    // 	  monotony_active = false;
    // 	  double coef = pred_h(_ode, atol);
    // 	  if (coef == 0.0)
    // 	    coef=1.0;
    // 	  double h = std::min(std::min(dynibex_hmax,0.01/coef),T/2.0);
    //
    // 	  embedded_ode->frechet_precomputing(order_of_method(meth));
    // 	  solution_j u_j0(*embedded_ode->yinit_aff,embedded_ode->t0,h,atol);
    // 	  list_solution_j.push_back(u_j0);
    // 	}

    /**constructor*/
    simulation(ivp_ode* _ode, double T,double a){
      process = 0;
      embedded_ode = _ode;
      time_T = T;
      atol = a;
      meth = RK4;
      nb_var=_ode->nbvar;
      monotony_active = false;
      double coef = pred_h(_ode, atol);
      if (coef == 0.0)
	coef=1.0;
      double h = std::min(std::min(dynibex_hmax,0.01/coef),T/2.0);

      if (embedded_ode->get_lte_computation() == SYMBOLIC){
	embedded_ode->frechet_precomputing(order_of_method(meth));
      } else {
	embedded_ode->dag_precomputing(order_of_method(meth));
      }
      solution_j u_j0(*embedded_ode->yinit_aff,embedded_ode->t0,h,atol);
      list_solution_j.push_back(u_j0);
    }


    /**constructor for dae*/
    simulation(ivp_dae_h1* _dae, double T,Method _meth, double a){
      process = 0;
      embedded_dae = _dae;
      time_T = T;
      atol = a;
      meth = RADAU3_DAE;
      nb_var=_dae->nbvar;
      monotony_active = false;
      double h = 0.001;

      embedded_dae->frechet_precomputing(order_of_method(meth));

      solution_j_dae u_j0(*embedded_dae->yinit_aff,*embedded_dae->xinit_aff,embedded_dae->t0,h,atol);
      u_j0.uniqueness(_dae);
      list_solution_j_dae.push_back(u_j0);
    }

    /**constructor for dae*/
    simulation(ivp_dae_h1* _dae, double T){
      process = 0;
      embedded_dae = _dae;
      time_T = T;
      atol = 1e-6;
      meth = RADAU3_DAE;
      nb_var=_dae->nbvar;
      monotony_active = false;
      double h = 0.001;
      embedded_dae->frechet_precomputing(order_of_method(meth));
      solution_j_dae u_j0(*embedded_dae->yinit_aff,*embedded_dae->xinit_aff,embedded_dae->t0,h,atol);
      u_j0.uniqueness(_dae);
      list_solution_j_dae.push_back(u_j0);
    }

    /**constructor for dae*/
    simulation(ivp_dae_h1* _dae, double T,Method _meth){
      process = 0;
      embedded_dae = _dae;
      time_T = T;
      atol = 1e-6;
      meth = RADAU3_DAE;
      nb_var=_dae->nbvar;
      monotony_active = false;
      double h = 0.001;
      embedded_dae->frechet_precomputing(order_of_method(meth));
      solution_j_dae u_j0(*embedded_dae->yinit_aff,*embedded_dae->xinit_aff,embedded_dae->t0,h,atol);
      u_j0.uniqueness(_dae);
      list_solution_j_dae.push_back(u_j0);
    }

    /**constructor for dae*/
    simulation(ivp_dae_h1* _dae, double T,double a){
      process = 0;
      embedded_dae = _dae;
      time_T = T;
      atol = a;
      meth = RADAU3_DAE;
      nb_var=_dae->nbvar;
      monotony_active = false;
      double h = 0.001;
      embedded_dae->frechet_precomputing(order_of_method(meth));
      solution_j_dae u_j0(*embedded_dae->yinit_aff,*embedded_dae->xinit_aff,embedded_dae->t0,h,atol);
      u_j0.uniqueness(_dae);
      list_solution_j_dae.push_back(u_j0);
    }


	/**********************************************/
	// Add by Alex in Sweden
	//constructor by copy
 simulation(const simulation& simu) : list_solution_g(), list_solution_j(), list_solution_j_dae(),
	  embedded_dae()
	  {

	      // Pointeur
	      embedded_ode = new ivp_ode (*simu.embedded_ode);

	      inverse_ode = new ivp_ode(embedded_ode, true, simu.time_T,
	      				*(simu.embedded_ode->yinit_aff));

	      // Pas pointeur
	      process = simu.process;
	      time_T = simu.time_T;
	      atol = simu.atol;
	      meth = simu.meth;
	      nb_var = simu.nb_var;
	      monotony_active = simu.monotony_active;

	      // Is it necessary ?
	      embedded_ode->frechet_precomputing(order_of_method(meth));

	      // Copy of the solution list
	      std::list<solution_g>::const_iterator iterator_list;
	      for(iterator_list = simu.list_solution_g.begin();
		  iterator_list != simu.list_solution_g.end();
		  iterator_list++)
		{
		  solution_g* sol_temp = new solution_g(iterator_list->time_j, new IntervalVector(*iterator_list->box_j1), new IntervalVector(*iterator_list->box_jn), new IntervalVector(*iterator_list->box_jnh), nb_var);
		  list_solution_g.push_back(*sol_temp);

		  delete sol_temp;
		}
	      if (! list_solution_j.empty ()) { std::cerr << "ERROR copy constructor simulation" << std::endl; }
	    }
	/**********************************************/

    //destructor
    ~simulation(){
      /*list_solution_j.clear();
	list_solution_j_dae.clear();
	list_solution_g.clear(); */
      destructor();
    }

    void destructor()
    {
      if (!list_solution_j_dae.empty())
	{
	  std::list<solution_j_dae>::iterator iterator_list;
	  for(iterator_list=list_solution_j_dae.begin();iterator_list!=list_solution_j_dae.end();iterator_list++)
	    {
	      iterator_list->destructor();
	    }
	}
      if (!list_solution_j.empty())
	{
	  std::list<solution_j>::iterator iterator_list;
	  for(iterator_list=list_solution_j.begin();iterator_list!=list_solution_j.end();iterator_list++)
	    {
	      iterator_list->destructor();
	    }
	}
      if (!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      iterator_list->destructor();
	    }
	}
	//add june 2020: zono tube
	if (!zonotopicard.empty())
	{
		zonotopicard.clear();
	}
	
    }


    /**return last state*/
    IntervalVector get_last()
    {
      if (!list_solution_g.empty())
	{
	  solution_g sol_temp = list_solution_g.back();
	  return *sol_temp.box_jnh;
	}
      else
	return *embedded_ode->yinit;
    }

    /**return last state in affine form*/
    Affine2Vector get_last_aff()
    {
      if (!failed)
	return *last_aff;
      else
	{
	  if (!list_solution_g.empty())
	    {
	      solution_g sol_temp = list_solution_g.back();
	      return Affine2Vector(*sol_temp.box_jnh,true);
	    }
	  else
	    {
	      return *embedded_ode->yinit_aff;
	    }
	}
    }

   /*******interval bool methods : BoolInterval = { EMPTY_BOOL, NO, YES, MAYBE } *****/
   /**final solution is include in a box*/
    BoolInterval finished_in_b(IntervalVector y_final)
    {
      if (y_final.size()!=nb_var)
	return EMPTY_BOOL;

      if(!list_solution_g.empty())
	{
	  solution_g sol_temp = list_solution_g.back();
          bool test = sol_temp.box_jnh->is_subset(y_final);
	  if (test)
	  	return YES;
	  else
		return MAYBE;
	}
      else
	return EMPTY_BOOL;
    }

    /**tube crosses a box*/
    BoolInterval has_crossed_b(IntervalVector y)
    {
      if (y.size()!=nb_var)
	return EMPTY_BOOL;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if (!((y_temp&y).is_empty()))
		return MAYBE;
	    }
	  return NO;
	}
      return EMPTY_BOOL;
    }

    /**test if tube crosses a box before a given time*/
    BoolInterval has_crossed_before_b(IntervalVector& y, double time)
    {
      //bool test = false;
      if (y.size()!=embedded_ode->nbvar)
	return EMPTY_BOOL;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;

	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      if (iterator_list->time_j.ub() < time)
		{
		  IntervalVector y_temp = *(iterator_list->box_j1);
		  if (!((y_temp&y).is_empty()))
		    {
		      return MAYBE;
		    }
		}
	      else
		return NO;
	    }
	  return NO;
	}
      else
	return EMPTY_BOOL;
    }



    /**tube stays in a box*/
    BoolInterval stayed_in_b(IntervalVector y_hull)
    {
      if (y_hull.size()!=nb_var)
	return EMPTY_BOOL;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if ((y_temp&y_hull).is_empty())
		return NO;
	      if (!y_temp.is_subset(y_hull))
		return MAYBE;
	    }
	  return YES;
	}
      return EMPTY_BOOL;
    }


    /**at least one y(t) outside a box*/
    BoolInterval go_out_b(IntervalVector y_hull)
    {
      if (y_hull.size()!=nb_var)
	return EMPTY_BOOL;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_jnh);
	      if ((y_temp&y_hull).is_empty())
		return YES;
	    }
	  return MAYBE;
	}
      return EMPTY_BOOL;
    }

    /**tube stays in a box till t*/
    BoolInterval stayed_in_till_b(IntervalVector y_hull, double t)
    {
      if (y_hull.size()!=nb_var)
	return EMPTY_BOOL;

      if (t <0)
	return MAYBE;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if (iterator_list->time_j.lb() > t)
		return YES;
	      if ((y_temp&y_hull).is_empty())
		return NO;
	      if (!y_temp.is_subset(y_hull))
		return MAYBE;
	    }
	  return YES;
	}
      return EMPTY_BOOL;
    }

    /**final solution crosses a box*/
    BoolInterval has_reached_b(IntervalVector y_final)
    {
      if (y_final.size()!=nb_var)
	return EMPTY_BOOL;

      if(!list_solution_g.empty())
	{
	  solution_g sol_temp = list_solution_g.back();
	  bool test = (!((*sol_temp.box_jnh)&y_final).is_empty());
	  if (test)
		return YES; 
	  else
	  	return NO;
	}
      else
	return MAYBE;

    }


  /*******end of interval bool methods*****/



    /**final solution is include in a box*/
    bool finished_in(IntervalVector y_final)
    {
      if (y_final.size()!=nb_var)
	return false;

      if(!list_solution_g.empty())
	{
	  solution_g sol_temp = list_solution_g.back();
	  return sol_temp.box_jnh->is_subset(y_final);
	}
      else
	return false;
    }

    /**final solution is include in at least one box of a list*/
    bool finished_in(std::list<IntervalVector> *stack)
    {
      std::list<IntervalVector>::iterator iterator_l;
      for(iterator_l=stack->begin();iterator_l!=stack->end();iterator_l++)
	{
	  if (finished_in((IntervalVector)*iterator_l))
	    return true;
	}
      return false;
    }


    /**tube crosses a box*/
    bool has_crossed(IntervalVector y)
    {
      if (y.size()!=nb_var)
	return false;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if (!((y_temp&y).is_empty()))
		return true;
	    }
	  return false;
	}
      return false;
    }

    /**give first instant when tube crosses a box y*/
    Interval has_crossed_when(IntervalVector& y)
    {
      if (y.size()!=embedded_ode->nbvar)
	return Interval(-1.0);

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;

	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if (!((y_temp&y).is_empty()))
		{
		  return iterator_list->time_j;
		}
	    }
	  return Interval(-1.0);
	}
      else
	return Interval(-1.0);
    }

    /**test if tube crosses a box before a given time*/
    bool has_crossed_before(IntervalVector& y, double time)
    {
      bool test = false;
      if (y.size()!=embedded_ode->nbvar)
	return test;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;

	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      if (iterator_list->time_j.ub() < time)
		{
		  IntervalVector y_temp = *(iterator_list->box_j1);
		  if (!((y_temp&y).is_empty()))
		    {
		      return true;
		    }
		}
	    }
	  return false;
	}
      else
	return test;
    }



    /**tube stays in a box*/
    bool stayed_in(IntervalVector y_hull)
    {
      if (y_hull.size()!=nb_var)
	return false;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if (!y_temp.is_subset(y_hull))
		return false;
	    }
	  return true;
	}
      return true;
    }


    /**at least one y(t) outside a box*/
    bool go_out(IntervalVector y_hull)
    {
      if (y_hull.size()!=nb_var)
	return false;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_jnh);
	      if ((y_temp&y_hull).is_empty())
		return true;
	    }
	  return false;
	}
      return false;
    }

    /**tube stays in a box till t*/
    bool stayed_in_till(IntervalVector y_hull, double t)
    {
      if (y_hull.size()!=nb_var)
	return false;

      if (t <0)
	return true;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      IntervalVector y_temp = *(iterator_list->box_j1);
	      if (iterator_list->time_j.lb() > t)
		return true;
	      if (!y_temp.is_subset(y_hull))
		return false;
	    }
	  return true;
	}
      return true;
    }

    /**final solution crosses a box*/
    bool has_reached(IntervalVector y_final)
    {
      if (y_final.size()!=nb_var)
	return false;

      if(!list_solution_g.empty())
	{
	  solution_g sol_temp = list_solution_g.back();
	  return (!((*sol_temp.box_jnh)&y_final).is_empty());
	}
      else
	return false;

    }


    /**final solution crosses at least one box of a list*/
    bool has_reached(std::list<IntervalVector> *stack)
    {
      std::list<IntervalVector>::iterator iterator_l;
      for(iterator_l=stack->begin();iterator_l!=stack->end();iterator_l++)
	{
	  if (has_reached(*iterator_l))
	    return true;
	}
      return false;
    }


    /**one solution is inside at least one box of a list*/
    double one_in(std::list<IntervalVector> *stack)
    {
      std::list<IntervalVector>::iterator iterator_l;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {

	      IntervalVector y_temp = *iterator_list->box_jnh;
	      double t= iterator_list->time_j.ub();

	      for(iterator_l=stack->begin();iterator_l!=stack->end();iterator_l++)
		{
		  if (y_temp.is_subset(*iterator_l))
		    return t;
		}
	    }
	  return (-1.0);
	}
      return (-1.0);
    }


    /**return a box containing y(t)*/
    IntervalVector get(double t)
    {

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      if ((iterator_list->time_j).contains(t))
		return *iterator_list->box_j1;
	    }
	  return IntervalVector(nb_var);
	}
      return IntervalVector(nb_var);

    }

    /**return [y([t])]*/
    IntervalVector get(Interval t)
    {

      IntervalVector hull(nb_var);
      bool first_time=true;

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      if (iterator_list->time_j.lb() >= t.ub())
		return hull;
	      //if (!(iterator_list->time_j & t).is_empty())
	      if (iterator_list->time_j.ub() > t.lb())
		{
		  if (first_time)
		    {
		      hull = *iterator_list->box_j1;
		      first_time = false;
		    }
		  else
		    hull |= *iterator_list->box_j1;
		}
	    }

	}
      return hull;

    }

    /**return a tight box containing y(t)*/
    IntervalVector get_tight(double t)
    {

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      if ((iterator_list->time_j).contains(t))
		{
		  Affine2Vector* it_box_jn_aff = new Affine2Vector(*iterator_list->box_jn,true);
		  double it_tn = iterator_list->time_j.lb();
		  double it_h = t - it_tn;
                  
		  switch(meth)
		    {
		    case IEULER:
		      {
			solution_j_ieuler u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,1);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case IMIDPOINT:
		      {
			solution_j_imidpoint u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,1);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case RADAU3:
		      {
			solution_j_radau3 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case HEUN:
		      {
			solution_j_heun u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,1);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case TAYLOR4:
		      {
			solution_j_tayl4 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case LA3:
		      {
			solution_j_la3 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case LC3:
		      {
			solution_j_lc3 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);


			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case RK4:
		      {
			solution_j_rk4 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);


			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case RADAU5:
		      {
			solution_j_radau5 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,3);



			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case GL4:
		      {
			solution_j_gauss4 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);


			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case GL6:
		      {
			solution_j_gauss6 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,3);


			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }
		    case KUTTA3:
		      {
			solution_j_kutta3 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
			u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

			//insert the new solution
			solution_g sol_temp(u_j.time_j, u_j.box_j1,u_j.box_jn, u_j.box_jnh, // u_j.box_jn_aff, u_j.box_jnh_aff,
					    nb_var); // Add affine forms
			list_solution_g.insert(iterator_list,sol_temp);
			iterator_list->time_j = Interval((u_j.time_j).ub(),iterator_list->time_j.ub());
			*iterator_list->box_jn = *u_j.box_jnh;
			*iterator_list->box_j1 &= iterator_list->picard(*iterator_list->box_j1,embedded_ode,2);

			if (u_j.time_j.ub() == t)
			  return *u_j.box_jnh;
			else
			  return *u_j.box_j1;
			break;
		      }

		    default:
		      return get(t);
		    }
		}
	    }
	  return IntervalVector(nb_var);
	}
      return IntervalVector(nb_var);

    }

    /**return a box contained in the next box (possibly an attractor)*/
    IntervalVector get_attractor()
    {

      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  iterator_list=list_solution_g.begin();
	  IntervalVector attractor = *iterator_list->box_jnh;

	  iterator_list++; //prob 2 val idem

	  for(;iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      if (attractor.is_subset(*iterator_list->box_jnh))
		return attractor;
	      else
		attractor = *iterator_list->box_jnh;
	    }
	  return IntervalVector(nb_var);
	}

      return IntervalVector(nb_var);

    }

    /**return the domain covered by the simulation (the hull of y)*/
    IntervalVector get_domain()
    {
      if(!list_solution_g.empty())
	{
	  std::list<solution_g>::iterator iterator_list;
	  iterator_list=list_solution_g.begin();

	  IntervalVector hull = *iterator_list->box_jnh;

	  for(;iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      hull |= *iterator_list->box_jnh;
	    }
	  return hull;
	}

      return IntervalVector(nb_var);

    }


	Affine2Vector contract_step(std::list<solution_g>::iterator iterator_list, ivp_ode* ode, Affine2Vector jn)
	{
	  *iterator_list->box_jn&=jn.itv();
	  Interval time_j = iterator_list->time_j;
	  //std::cout << "contract j1: sol" << std::endl;
	  //TODO choice of meth
	  solution_j_rk4 u_j(jn,time_j.lb(),time_j.diam(), ode,atol,1.0);
	  *u_j.box_j0 = *iterator_list->box_j1;
	  *u_j.box_j1 = *iterator_list->box_j1;
	  *u_j.box_jn = *iterator_list->box_jn;
	  *u_j.box_jnh = *iterator_list->box_jnh;
	  
	  //std::cout << "contract j1: picard" << std::endl;
	  //std::cout << "contract j1 av: " << *iterator_list->box_j1 << std::endl;
	  *iterator_list->box_j1 &= u_j.picard(*iterator_list->box_j1,ode,2);
	  //std::cout << "contract j1 ap: " << *iterator_list->box_j1 << std::endl;
	  
	  //std::cout << "contract jnh av: " << *u_j.box_jnh << std::endl;
	  *u_j.box_err_aff = u_j.LTE(*iterator_list->box_j1,ode, time_j.diam());
	  
	  u_j.calcul_jnh(ode);
	  
	  //std::cout << "new jnh : " << u_j.box_jnh_aff->itv() << std::endl;
	  //TODO: to avoid empty, sure ?
	  if (!(*iterator_list->box_jnh & u_j.box_jnh_aff->itv()).is_empty())
		*iterator_list->box_jnh &= u_j.box_jnh_aff->itv();
	  //std::cout << "contract jnh ap: " << *iterator_list->box_jnh << std::endl;
	  
	  return Affine2Vector(*iterator_list->box_jnh,true);//*u_j.box_jnh_aff;
	}
	
	Affine2Vector contract_step_bwd(std::list<solution_g>::iterator iterator_list, ivp_ode* ode, Affine2Vector jnh)
	{
	  *iterator_list->box_jnh&=jnh.itv();
	  Interval time_j = iterator_list->time_j;
	  //std::cout << "contract bwd from : " << jnh.itv() << std::endl;
	  //TODO choice of meth
	  solution_j_rk4 u_j(jnh,time_j.ub(),time_j.diam(), ode,atol,1.0);
	  *u_j.box_j0 = *iterator_list->box_j1;
	  *u_j.box_j1 = *iterator_list->box_j1;
	  *u_j.box_jn = *iterator_list->box_jnh;
	  *u_j.box_jnh = *iterator_list->box_jn;
	  
	  //std::cout << "contract bwd from : " << *u_j.box_jn << std::endl;
	  
	  //std::cout << "contract j1: picard" << std::endl;
	  //std::cout << "contract j1 av: " << *iterator_list->box_j1 << std::endl;
	  *iterator_list->box_j1 &= u_j.picard(*iterator_list->box_j1,ode,2);
	  //std::cout << "contract j1 ap: " << *iterator_list->box_j1 << std::endl;
	  
	  //std::cout << "contract jnh av: " << *u_j.box_jnh << std::endl;
	  *u_j.box_err_aff = u_j.LTE(*iterator_list->box_j1,ode, time_j.diam());
	  
	  u_j.calcul_jnh(ode);
	  
	  //std::cout << "old jn : " << *iterator_list->box_jn << std::endl;
	  //std::cout << "new jnh : " << u_j.box_jnh_aff->itv() << std::endl;
	  //TODO: to avoid empty, sure ?
	  if (!(*iterator_list->box_jn & u_j.box_jnh_aff->itv()).is_empty())
		*iterator_list->box_jn &= u_j.box_jnh_aff->itv();
	  //std::cout << "contract jnh ap: " << *iterator_list->box_jnh << std::endl;
	  
	  return Affine2Vector(*iterator_list->box_jn,true);//*u_j.box_jnh_aff;
	}
	

	//propagate a contraction on initial state (or parameters included in initial state)
	void propag(IntervalVector yinit)
	{

	  if((!list_solution_g.empty()) && (embedded_ode != NULL))
	  {
	      std::list<solution_g>::iterator iterator_list;
	      iterator_list=list_solution_g.begin();
	      *iterator_list->box_jn &= yinit;
	      *iterator_list->box_jnh &= yinit;
	      *iterator_list->box_j1 &= yinit;

	      //std::cout << "propag" << std::endl;
	      Affine2Vector temp = Affine2Vector(*iterator_list->box_jnh,true);

	      iterator_list++;
	      for(;iterator_list!=list_solution_g.end();iterator_list++)
	      {
		//std::cout << "jn av: " << *iterator_list->box_jn << std::endl;
		*iterator_list->box_jn &= temp.itv();
		//std::cout << "jn ap: " << *iterator_list->box_jn << std::endl;

		temp = contract_step(iterator_list,embedded_ode,temp);
	      }

	  }
	 return;
	}

	//contract y(t) w.r.t. y(t) \in A
	IntervalVector contract(double t, IntervalVector A)
	{
	    assert(A.size() == nb_var);

	    if((!list_solution_g.empty()) && (embedded_ode != NULL))
	    {
	      std::list<solution_g>::iterator iterator_list;
	      for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	      {
		if ((iterator_list->time_j).contains(t))
		{
		  //std::cout << "t found" << std::endl;

		  //left of t
		  Affine2Vector* it_box_jn_aff = new Affine2Vector(*iterator_list->box_jn,true);
		  double it_tn = iterator_list->time_j.lb();
		  double it_h = t - it_tn;


		  //TODO choice of meth
		  solution_j_rk4 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
		  u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

		  //contraction
		  IntervalVector jt(*u_j.box_jnh);

		  //std::cout << "jt avant ctc : " << jt << std::endl;

		  jt &= A;

		  //std::cout << "jt apres ctc : " << jt << std::endl;

		  solution_g u_g(u_j.time_j,u_j.box_j1,u_j.box_jn,&jt,nb_var);

		  //right of t
		  Interval time_j_2(t,iterator_list->time_j.ub());

		  solution_g u_g2(time_j_2,iterator_list->box_j1,&jt,iterator_list->box_jnh,nb_var);

		  //add new steps
		  list_solution_g.insert(iterator_list,u_g);
		  //iterator_list++;
		  list_solution_g.insert(iterator_list,u_g2);

		  //delete of past step
		  iterator_list = list_solution_g.erase(iterator_list);

		  return jt;
		}
	      }
	    }
	  return A; ///ça peut foutre la merde j'ai pas testé  
	}


	//contract y(t) w.r.t. ctc(y(t))
	IntervalVector contract(double t, CtcFwdBwd* ctc)
	{

	    if((!list_solution_g.empty()) && (embedded_ode != NULL))
	    {
	      std::list<solution_g>::iterator iterator_list;
	      for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	      {
		if ((iterator_list->time_j).contains(t))
		{
		  //std::cout << "t found" << std::endl;

		  //left of t
		  Affine2Vector* it_box_jn_aff = new Affine2Vector(*iterator_list->box_jn,true);
		  double it_tn = iterator_list->time_j.lb();
		  double it_h = t - it_tn;


		  //TODO choice of meth
		  solution_j_rk4 u_j(*it_box_jn_aff,it_tn,it_h, embedded_ode,atol,1.0);
		  u_j.compute_oneStep(*it_box_jn_aff,embedded_ode,monotony_active);

		  //contraction
		  IntervalVector jt(*u_j.box_jnh);

		  //std::cout << "jt avant ctc : " << jt << std::endl;

		  ctc->contract(jt);
		  //jt &= A;

		  //std::cout << "jt apres ctc : " << jt << std::endl;

		  solution_g u_g(u_j.time_j,u_j.box_j1,u_j.box_jn,&jt,nb_var);

		  //right of t
		  Interval time_j_2(t,iterator_list->time_j.ub());

		  solution_g u_g2(time_j_2,iterator_list->box_j1,&jt,iterator_list->box_jnh,nb_var);

		  //add new steps
		  list_solution_g.insert(iterator_list,u_g);
		  //iterator_list++;
		  list_solution_g.insert(iterator_list,u_g2);

		  //delete of past step
		  iterator_list = list_solution_g.erase(iterator_list);

		  return jt;
		}
	      }
	    }
	}
	
	
	//propagate in forward after t
	double propag_fwd(double t)
	{
	  double gain = 0.0;

	  if((!list_solution_g.empty()) && (embedded_ode != NULL))
	  {
	     std::list<solution_g>::iterator iterator_list;
	     bool propag=false;
	     Affine2Vector temp = Affine2Vector(*list_solution_g.begin()->box_jn,true);
	    IntervalVector old_temp = *list_solution_g.begin()->box_jnh; for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	      {
		old_temp = *iterator_list->box_jnh;
		if (!propag)
		  temp = Affine2Vector(*iterator_list->box_jn,true);

		if (((iterator_list->time_j).contains(t)) || (iterator_list->time_j.ub() == t))
		{
		   propag=true;
		}
		if (propag)
		{
		  temp = contract_step(iterator_list,embedded_ode,temp);
		  gain = std::max(gain, temp.itv().rel_distance(old_temp));
		}
	      }
	  }
	 return gain;
	}

	// propag bwd
	double propag_bwd(double t)
	{
	  double gain = 0.0;
	  if((!list_solution_g.empty()) && (inverse_ode != NULL))
	  {
	    //std::cout << *inverse_ode->ydot << std::endl;

	    std::list<solution_g>::iterator iterator_list=list_solution_g.end();
	    iterator_list--;
	    bool propag=false;
	    Affine2Vector temp = Affine2Vector(*(iterator_list->box_jnh),true);
	    IntervalVector old_temp = *iterator_list->box_jn;
	    //std::cout << temp.itv() << std::endl;
	    for(;iterator_list!=list_solution_g.begin();iterator_list--)
	      {
		old_temp = *iterator_list->box_jn;
		//std::cout << "old jn : " << *iterator_list->box_jn << std::endl;
		if (!propag)
		  temp = Affine2Vector(*iterator_list->box_jnh,true);

		if (((iterator_list->time_j).contains(t)) || (iterator_list->time_j.lb() == t))
		{
		   propag=true;
		}
		if (propag)
		{
		  temp = contract_step_bwd(iterator_list,inverse_ode,temp);
		  gain = std::max(gain, temp.itv().rel_distance(old_temp));
		  //std::cout << "new jn : " << temp.itv() << std::endl;
		}
	      }
	      temp = contract_step_bwd(iterator_list,inverse_ode,temp);
	      //gain = std::max(gain, temp.itv().rel_distance(old_temp));
	  }
	 return gain;

	}


	//fixed point
	void fixed_point(double fix_tol)  //TODO: hard test !
	{
	  double gain=propag_fwd(embedded_ode->t0);
	  while (gain > fix_tol)
	  {
	    gain = std::max(propag_bwd(time_T),propag_fwd(embedded_ode->t0));
	    //std::cout << "gain : " << gain << std::endl;
	    /* gain = std::max(gain,propag_fwd(embedded_ode->t0);
	    std::cout << "gain : " << gain << std::endl;*/
	  }
	}


    /**export in a file for ploting*/
    void export2d_yn(const char* filename, int x, int y)
    {
      assert(nb_var > 1);
      std::cout << "export in progress..." << std::endl;
      if(!list_solution_g.empty())
	{
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      file << iterator_list->box_jnh->operator[](x) << " ; " << iterator_list->box_jnh->operator[](y) << std::endl;
	    }

	  file.close();
	}

    };

    /**export in a file for ploting*/
    void export3d_yn(const char* filename, int x, int y, int z)
    {
      assert(nb_var > 2);
      std::cout << "export in progress..." << std::endl;
      if(!list_solution_g.empty())
	{
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      file << iterator_list->box_jnh->operator[](x) <<
		" ; " << iterator_list->box_jnh->operator[](y) <<
		" ; " << iterator_list->box_jnh->operator[](z) << std::endl;
	    }


	  file.close();
	}

    };

    /**export in a file for ploting*/
    void export1d_yn(const char* filename, int x)
    {
      assert(nb_var > 1);
      std::cout << "export in progress..." << std::endl;
      if(!list_solution_g.empty())
	{
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      file << iterator_list->box_jnh->operator[](x) <<
		" ; " << iterator_list->time_j << std::endl;
	    }

	  file.close();
	}

    };

    /**export all dimensions off picard boxes wrt time*/
    void export_y0(const char* filename)
    {
      assert(nb_var > 1);
      std::cout << "export in progress..." << std::endl;
      if(!list_solution_g.empty())
	{
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      file << iterator_list->time_j <<
		" ; " << *iterator_list->box_j1 << std::endl;
	    }

	  file.close();
	}

    };

    /**export all dimensions off discretized boxes wrt time*/
    void export_yn(const char* filename)
    {
      assert(nb_var > 1);
      std::cout << "export in progress..." << std::endl;
      if(!list_solution_g.empty())
	{
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	      file << iterator_list->time_j.ub() <<
		" ; " << *iterator_list->box_jnh << std::endl;
	    }

	  file.close();
	}

    };

    void generate_python_plot(const char* filename, int x, int y)
    {
    	if(!list_solution_g.empty())
	{
	int i=0;
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  
	  file << "#!/usr/bin/env python\nfrom __future__ import print_function\nimport sys\nimport numpy as np\nimport matplotlib.pyplot as plt\nimport polytope" << std::endl;
	  
	  std::list<solution_g>::iterator iterator_list;
	  for(iterator_list=list_solution_g.begin();iterator_list!=list_solution_g.end();iterator_list++)
	    {
	    file << "V=np.array([";
	      file << "["<< iterator_list->box_j1->operator[](x).lb() << " , " << iterator_list->box_j1->operator[](y).lb() << "]," << std::endl;
	      file << "["<< iterator_list->box_j1->operator[](x).ub() << " , " << iterator_list->box_j1->operator[](y).lb() << "]," << std::endl;
	      file << "["<< iterator_list->box_j1->operator[](x).lb() << " , " << iterator_list->box_j1->operator[](y).ub() << "]," << std::endl;
	      file << "["<< iterator_list->box_j1->operator[](x).ub() << " , " << iterator_list->box_j1->operator[](y).ub() << "]" << std::endl;
	      file << "])\nP"<<i<<" = polytope.qhull(V)\n";
	    	i++;
	    }
	    
	    file << "reg=[";
	    for (int j=0;j<i;j++)
	    {
	    	if (j>0)
	    		file << ",P"<<j;
	    	else
	    		file << "P"<<j;
	    }
	    file << "]\nr=polytope.Region(reg)\nf,a = plt.subplots(dpi=200)\na = r.plot(ax=a, linewidth=0.5, linestyle='dashed', color='blue', edgecolor='black')\n";
	    //r.plot()\nplt.setp(plt.gca().patch,linestyle='--',linewidth=1)\n";
	    
	    file << "plt.autoscale(True)\nplt.show()" << std::endl; 

	  file.close();
	}
    };

void generate_python_plot_zono(const char* filename, int x, int y,double prec, int noise_number)
    {
    	if(!zonotopicard.empty())
	{
	int i=0;
	  std::ofstream file(filename, std::ios::out | std::ios::trunc);
	  
	  file << "#!/usr/bin/env python\nfrom __future__ import print_function\nimport sys\nimport numpy as np\nimport matplotlib.pyplot as plt\nimport polytope" << std::endl;

	    std::list<Affine2Vector>::iterator iterator_list;
	    AF_fAFFullI::setAffineNoiseNumber(noise_number);
	    for(iterator_list=zonotopicard.begin();iterator_list!=zonotopicard.end();iterator_list++)
	    {
		Affine2Vector temp = *iterator_list;
		temp.compact(prec); 
		std::list<std::vector<double> > lv = toVertices(temp);

		file << "V=np.array([";
  		std::list<std::vector<double> >::const_iterator it;
  		for(it=lv.begin();it!=lv.end();it++) 
  		{
  			file << *it << "," << std::endl;
 		 }
  		file << "])" << std::endl << "P"<<i<<" = polytope.qhull(V)"<< std::endl ;
		i++;

   	    }
	    
	    file << "reg=[";
	    for (int j=0;j<i;j++)
	    {
	    	if (j>0)
	    		file << ",P"<<j;
	    	else
	    		file << "P"<<j;
	    }
	    file << "]\nr=polytope.Region(reg)\nf,a = plt.subplots(dpi=200)\na = r.plot(ax=a, linewidth=0.5, linestyle='dashed', color='blue', edgecolor='black')\n";
	    //r.plot()\nplt.setp(plt.gca().patch,linestyle='--',linewidth=1)\n";
	    
	    file << "plt.autoscale(True)\nplt.show()" << std::endl; 

	  file.close();
	}
    };

   void export_zono_tube_py(const char* filename, double prec, int noise_number) // prec on zonotope definition
   {
   	if (!zonotopicard.empty())
        {

	    std::ofstream file(filename, std::ios::out | std::ios::trunc);
	    file << "#!/usr/bin/env python\nfrom __future__ import print_function\nimport sys\nimport numpy as np\nimport matplotlib.pyplot as plt\nimport polytope" << std::endl;

  	    //file << "list_zono=[]" << std::endl;

	    int k=0;

	    std::list<Affine2Vector>::iterator iterator_list;
	    AF_fAFFullI::setAffineNoiseNumber(noise_number);
	    for(iterator_list=zonotopicard.begin();iterator_list!=zonotopicard.end();iterator_list++)
	    {
		Affine2Vector temp = *iterator_list;
		temp.compact(prec); 
		std::list<std::vector<double> > lv = toVertices(temp);

		file << "V=np.array([";
  		std::list<std::vector<double> >::const_iterator it;
  		for(it=lv.begin();it!=lv.end();it++) 
  		{
  			file << *it << "," << std::endl;
 		 }
  		file << "])" << std::endl << "P = polytope.qhull(V)"<< std::endl ;
		file << "print(P.vertices)" << std::endl;
		//file << "list_zono.append(P)" << std::endl;
		k++;
		//if (k > 10)
		//	break;
   	    }
	    //file << "print(list_zono)" << std::endl;

   	    file.close();

	}
	else
		std::cout << "Empty tube !" << std::endl;

   }

  };





  //}
}

#endif
