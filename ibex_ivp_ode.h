/* ============================================================================
 * D Y N I B E X - Definition of the ODE structure
 * ============================================================================
 * Copyright   : ENSTA ParisTech
 * License     : This program can be distributed under the terms of the GNU LGPL.
 *               See the file COPYING.LESSER.
 *
 * Author(s)   : Julien Alexandre dit Sandretto and Alexandre Chapoutot
 * Created     : Jul 18, 2014
 * Author(s) of modification   : Julien Alexandre dit Sandretto and Olivier Mullier
 * Modified    : Jul 25, 2018
 * Sponsored   : This research benefited from the support of the "Chair Complex Systems Engineering - Ecole Polytechnique, THALES, DGA, FX, DASSAULT AVIATION, DCNS Research, ENSTA ParisTech, Telecom ParisTech, Fondation ParisTech and FDO ENSTA"
 * ---------------------------------------------------------------------------- */
#ifndef IBEX_ODE_H
#define IBEX_ODE_H

const int MAX_FCT=60;
const double tol_ctc=1e-15;

//define which method is used to compute the LTE (nbvar >= __MAX_DIM_SWITCH__ means automatic differentiation is used, symbolic method otherwise).
#define __MAX_DIM_SWITCH__ 3

namespace ibex{

enum Lte_type {AUTODIF, SYMBOLIC, AUTO};

class ivp_ode
{
public:
  ivp_ode(const Function _ydot, double _t0, const IntervalVector _yinit, Lte_type _lte_computation = AUTO){
      nbvar = _yinit.size();
      _dags = NULL;
      yinit = new IntervalVector(_yinit);
      yinit_aff = new Affine2Vector(_yinit,true);

      ydot = new Function(_ydot);
      t0 = _t0;
      embedded_ctc = NULL;
      if (_lte_computation == AUTO)
	lte_computation = lte_selection();
      else
	lte_computation = _lte_computation;
  };

  ivp_ode(const Function _ydot, double _t0, const Affine2Vector _yinit_aff, Lte_type _lte_computation = AUTO){
      nbvar = _yinit_aff.size();
      _dags = NULL;
      yinit = new IntervalVector(_yinit_aff.itv());
      yinit_aff = new Affine2Vector(_yinit_aff);

      ydot = new Function(_ydot);
      t0 = _t0;
      embedded_ctc = NULL;
  };

  ivp_ode(ivp_ode* _ode, bool inverse, double _tf, const Affine2Vector _yinit_aff){
    nbvar = _ode->nbvar;
    yinit = new IntervalVector(_yinit_aff.itv());
    yinit_aff = new Affine2Vector(_yinit_aff);

    Function f=*_ode->ydot;

    //std::cout << "old func : " << f << std::endl;

    Array <const ExprSymbol> args(f.nb_arg());
    varcopy (f.args(), args);
    const ExprConstant* cst = dynamic_cast<const ExprConstant*> (&f.expr());
    if (cst)
    {
      if (cst -> dim.is_vector ())
	ydot = new Function (args, -cst->get_vector_value());
      else
	ydot = new Function(args, -cst->get_matrix_value());
    }
    else
    {
      const ExprVector* vec = dynamic_cast<const ExprVector*>(&f.expr());
      assert(vec); // TODO: manage matrix-valued function
      Array<const ExprNode> minus_vec(vec -> nb_args);
      for (int i = 0; i < vec -> nb_args; i++)
      {
	minus_vec.set_ref (i, -ExprCopy().copy(f.args(), args, vec->arg(i)));
      }

      ydot = new Function (args, ExprVector::new_(minus_vec,vec->row_vector()));
    }

    //std::cout << "inverse func : " << *ydot << std::endl;

    //ydot = new Function(-(*_ode->ydot));
    t0 = _tf;
    embedded_ctc = NULL;

  };


  //with additive constraints
  ivp_ode(const Function _ydot, double _t0, const IntervalVector _yinit, const Array<NumConstraint>& csp, Lte_type _lte_computation = AUTO){
      nbvar = _yinit.size();
      _dags = NULL;
      yinit = new IntervalVector(_yinit);
      yinit_aff = new Affine2Vector(_yinit,true);

      ydot = new Function(_ydot);
      t0 = _t0;
      Array<NumConstraint> csp_temp(csp);
      CtcHC4* hc4 = new CtcHC4(csp_temp,tol_ctc);
      Ctc3BCid* cid = new Ctc3BCid(*hc4);
      embedded_ctc = new CtcFixPoint(*cid,tol_ctc);
      if (_lte_computation == AUTO)
	lte_computation = lte_selection();
      else
	lte_computation = _lte_computation;
  };

  ivp_ode(const Function _ydot, double _t0, const Affine2Vector _yinit_aff, const Array<NumConstraint>& csp, Lte_type _lte_computation = AUTO){
      nbvar = _yinit_aff.size();
      _dags = NULL;
      yinit = new IntervalVector(_yinit_aff.itv());
      yinit_aff = new Affine2Vector(_yinit_aff);

      ydot = new Function(_ydot);
      t0 = _t0;
      Array<NumConstraint> csp_temp(csp);
      CtcHC4* hc4 = new CtcHC4(csp_temp,tol_ctc);
      Ctc3BCid* cid = new Ctc3BCid(*hc4);
      embedded_ctc = new CtcFixPoint(*cid,tol_ctc);
      if (_lte_computation == AUTO)
	lte_computation = lte_selection();
      else
	lte_computation = _lte_computation;
  };


  /************** */
  /* Copy constructor */
  /* Warn: share pointers !!! */
  ivp_ode (const ivp_ode &source) {
    nbvar = source.nbvar;
    yinit = new IntervalVector(*source.yinit);
    yinit_aff = new Affine2Vector(*yinit, true);
    ydot = source.ydot;
    t0 = source.t0;
    embedded_ctc = source.embedded_ctc;
  }
  /***************/

  void frechet_precomputing(int order)
  {
//     cout << "ordre pour frechet : " << order << endl;
    edtfr = new edtree_frechet(order,ydot,nbvar);
  }


  IntervalVector compute_derivatives(int ordre, IntervalVector yi){
/*#ifdef __PRINT_DEBUG__
cout << __func__ << endl;cout.flush();
cout << "ordre : " << ordre << endl;cout.flush();
cout << "yi : " << yi << endl;cout.flush();
#endif*/
    assert(ordre < MAX_FCT);
    if (ordre == 1)
    {
        IntervalMatrix J(yi.size(), yi.size());
        ydot->hansen_matrix(yi,J);
        return (ydot->eval_vector(yi.mid()) + J*(yi-yi.mid())) & ydot->eval_vector(yi);
    }
    switch (lte_computation){
        case SYMBOLIC:
        {
            IntervalVector der(nbvar);
            for (unsigned int j = 0; j < nbvar; j++)
            der[j]=edtfr->get_derivatives(ordre,Affine2Vector(yi,true),j).itv();
            return der;
        }
        case AUTODIF:
        {
            Affine2Vector result = _dags->get_derivatives(ordre, Affine2Vector(yi, true));
            return result.itv();
        }
        default:
        {
            if (nbvar >= __MAX_DIM_SWITCH__)
                lte_computation = AUTODIF;
            else
                lte_computation = SYMBOLIC;
            return compute_derivatives(ordre, yi);
        }
    }
  };



  Affine2Vector computeRK4derivative(Affine2Vector y)
  {
    Affine2Vector rk4_deriv(y.size(),0);

    for (int i=0;i<y.size();i++)
    {
      rk4_deriv[i]=edtfr->lteExplicitRK4(i, y);

    }
    return rk4_deriv;

  }

  Affine2Vector computeGL4derivative_aff(Affine2Vector y)
  {
    Affine2Vector gl4_deriv(y.size());

    for (int i=0;i<y.size();i++)
    {
      gl4_deriv[i]=edtfr->lteImplicitGauss4(i, y);

    }
    return gl4_deriv;
  }

  Affine2Vector computeGL6derivative_aff(Affine2Vector y)
  {
    Affine2Vector gl6_deriv(y.size());

    for (int i=0;i<y.size();i++)
    {
      gl6_deriv[i]=edtfr->lteImplicitGauss6(i, y);

    }
    return gl6_deriv;
  }


  Affine2Vector computeRADAU3derivative(Affine2Vector y)
  {
    Affine2Vector radau3_deriv(y.size(),0);

    for (int i=0;i<y.size();i++)
    {
      radau3_deriv[i]=edtfr->lteImplicitRadau3(i, y);
    }
    return radau3_deriv;

  }

  Affine2Vector computeRADAU5derivative(Affine2Vector y)
    {
      Affine2Vector radau5_deriv(y.size(),0);

      for (int i=0; i<y.size(); i++)
        {
	  radau5_deriv[i]=edtfr->lteImplicitRadau2a5(i, y);
        }
      return radau5_deriv;

    }

  Affine2Vector computeLC3derivative(Affine2Vector y)
  {
    Affine2Vector lc3_deriv(y.size(),0);

    for (int i=0;i<y.size();i++)
    {
      lc3_deriv[i]=edtfr->lteImplicitLobbato3c4(i, y);
    }
    return lc3_deriv;

  }


  Affine2Vector computeHEUNderivative(Affine2Vector y)
  {
    Affine2Vector heun_deriv(y.size(),0);

    for (int i=0;i<y.size();i++)
    {
      heun_deriv[i]=edtfr->lteExplicitHeun(i, y);
    }
    return heun_deriv;

  }

  Affine2Vector computeIEULERderivative(Affine2Vector y)
  {
    Affine2Vector ieuler_deriv(y.size(),0);

    for (int i=0;i<y.size();i++)
    {
      ieuler_deriv[i]=edtfr->lteImplicitEuler(i, y);

    }
    return ieuler_deriv;

  }


  Affine2Vector computeIMIDPOINTderivative(Affine2Vector y)
  {
    Affine2Vector imidpoint_deriv(y.size(),0);

    for (int i=0;i<y.size();i++)
    {
      imidpoint_deriv[i]=edtfr->lteImplicitMidpoint(i, y);

    }
    return imidpoint_deriv;

  }

   Affine2Vector computeLA3derivative_aff(Affine2Vector y)
  {
    Affine2Vector la3_deriv(y.size());

    for (int i=0;i<y.size();i++)
    {
      la3_deriv[i]=edtfr->lteImplicitLobbato3a4(i, y);

    }
    return la3_deriv;

  }

  Affine2Vector computeKUTTA3derivative_aff(Affine2Vector y)
  {
    Affine2Vector kutta3_deriv(y.size());

    for (int i=0;i<y.size();i++)
    {
      kutta3_deriv[i]=edtfr->lteexplicitkutta3(i, y);

    }
    return kutta3_deriv;
  }



    //with complete affine form
   Affine2Vector compute_derivatives_aff(int ordre, Affine2Vector yi){
    assert(ordre < MAX_FCT);
    switch (lte_computation){
      case SYMBOLIC:
      {
	Affine2Vector der(nbvar);
	edtfr->edfr->inc_global_step();
	for (unsigned int j=0;j<nbvar;j++)
	  der[j]=edtfr->get_derivatives(ordre,yi,j);
	return der;
      }
      case AUTODIF:
      {
        return _dags->get_derivatives(ordre, yi);
      }
      case AUTO:
      {
        if (nbvar >= __MAX_DIM_SWITCH__)
            lte_computation = AUTODIF;
        else
            lte_computation = SYMBOLIC;
        return compute_derivatives_aff(ordre, yi);
      }
      default:
      {
        lte_computation = AUTODIF;
        return compute_derivatives_aff(ordre, yi);
      }
    }
  };

  IntervalMatrix eval_jacobian_init()
  {
    return ydot->jacobian(*yinit);

  };

  void compute_derivatives(int ordre){
    assert(ordre < MAX_FCT);


    for (int i=0;i<=ordre;i++)
    {
	std::cout << "eval "<<i<<"^th diff" << std::endl;
	for (unsigned int j=0;j<nbvar;j++)
	    std::cout <<  edtfr->get_derivatives(i,Affine2Vector(*yinit,true),j).itv() << std::endl;
    }


  };


    void compute_derivatives2(int ordre){
    assert(ordre < MAX_FCT);

    for (int i=0;i<=ordre;i++)
    {
	std::cout << "eval "<<i<<"^th diff" << std::endl;
	for (unsigned int j=0;j<nbvar;j++)
	    std::cout << edtfr->get_derivatives(i,Affine2Vector(*yinit,true),j) << std::endl;
    }


  };


  ~ivp_ode(){
    delete yinit;
    delete yinit_aff;
    delete ydot;
    if (lte_computation == SYMBOLIC)
      delete edtfr;
    if (lte_computation == AUTODIF)
      delete _dags;

    if (embedded_ctc != NULL)
      delete embedded_ctc;
  };


public:
  unsigned int nbvar;
  Function* ydot;
  IntervalVector* yinit;
  Affine2Vector* yinit_aff;
  double t0;
  CtcFixPoint* embedded_ctc;

  Lte_type get_lte_computation(){return lte_computation;};
  void dag_precomputing(int order){
    _dags = new wDAG_aff(ydot, order);
  }

  wDAG_aff * get_dags(){return _dags;}

  void set_Butcher_table(IntervalMatrix A, IntervalVector b){
    if (!_dags->is_filled_in()){
      _dags->set_filled_in(true);
      _dags->set_A(A);
      _dags->set_b(b);
    }
  }

  Lte_type lte_selection(){
    if (nbvar >= __MAX_DIM_SWITCH__){
	return ibex::AUTODIF;
    } else {
      return ibex::SYMBOLIC;
    }
  }

  std::string lte_comp_to_string(){
    if (lte_computation == ibex::AUTODIF)
      return "automatic differentiation";
    if (lte_computation == ibex::SYMBOLIC)
      return "symbolic differentiation";
    return "unknown differentiation";
  }

private:
  edtree_frechet* edtfr;
  wDAG_aff * _dags;
  Lte_type lte_computation;


};


}


#endif
