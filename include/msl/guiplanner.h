//----------------------------------------------------------------------
//               The Motion Strategy Library (MSL)
//----------------------------------------------------------------------
//
// Copyright (c) University of Illinois and Steven M. LaValle.     
// All Rights Reserved.
// 
// Permission to use, copy, and distribute this software and its 
// documentation is hereby granted free of charge, provided that 
// (1) it is not a component of a commercial product, and 
// (2) this notice appears in all copies of the software and
//     related documentation. 
// 
// The University of Illinois and the author make no representations
// about the suitability or fitness of this software for any purpose.  
// It is provided "as is" without express or implied warranty.
//----------------------------------------------------------------------

#ifndef MSL_GUIPLANNER_H
#define MSL_GUIPLANNER_H

#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <signal.h>
#include <fx.h>
#include "gui.h"
#include "defs.h"
#include "planner.h"
#include "rrt.h"
#include "rcrrt.h"
#include "prm.h"
#include "fdp.h"
#include "util.h"

//! A rendering-independent GUI for the Planner classes

class GuiPlanner;
class MSLPlotWindow;

class MSLPlannerWindow : public FXMainWindow {
  FXDECLARE(MSLPlannerWindow)
protected:
  FXMenuBar*         menubar;
  FXMenuBar*         vcrbar;
  FXMenuBar*         buttonbar;
  FXMenuPane*        loadmenu;
  FXMenuPane*        savemenu;
  FXMenuPane*        plotmenu;
  FXMenuPane*        plannermenu;
  FXMatrix*          matrix;

  FXDataTarget       plannerdeltat_target;
  FXDataTarget       numnodes_target;
  FXDataTarget       drawindexx_target;
  FXDataTarget       drawindexy_target;
  FXDataTarget       animationtimescale_target;
  FXDataTarget       ambientlight_target;

  GuiPlanner*        GP;

public:
  MSLPlannerWindow() {}
  MSLPlannerWindow(GuiPlanner* gp);
  virtual ~MSLPlannerWindow();

  void create();
  void Restart();
  long onCmdTimer(FXObject*,FXSelector,void*);
  long GeneralHandler(FXObject*,FXSelector,void*);

  friend class GuiPlanner;
  friend class MSLPlotWindow;

};



class MSLPlotWindow : public FXDialogBox {

  // Macro for class hierarchy declarations
  FXDECLARE(MSLPlotWindow)
private:

  FXHorizontalFrame *contents;                // Content frame
  FXVerticalFrame   *canvasFrame;             // Canvas frame
  FXVerticalFrame   *buttonFrame;             // Button frame
  FXCanvas          *canvas;                  // Canvas to draw into
  int               indexx,indexy;
  
protected:
  MSLPlotWindow(){}

  MSLPlannerWindow* Owner;
  GuiPlanner* GP;

public:

  // Message handlers
  long onPaint(FXObject*,FXSelector,void*);
  long onCmdPrint(FXObject*,FXSelector,void*);
  
  MSLPlotWindow(MSLPlannerWindow* owner);

  void drawPage(FXDC& dc,FXint w,FXint h,FXint tx = 0,FXint ty = 0);

  // Messages for our class
  enum{
    ID_CANVAS=FXMainWindow::ID_LAST,
    ID_PRINT,
    ID_LAST
    };
};



class GuiPlanner: public FXApp, public Gui {
 protected:
  virtual void Init();
  virtual void CreateMenuWindow();

  MSLPlannerWindow* Window;
 public:
  virtual void HandleEvents();
  virtual void ButtonHandle(int b);
  double LineWidth;
  double PSLineWidth;
  int DrawIndexX,DrawIndexY;
  Planner *Pl;
  GuiPlanner(Render *render, Planner *planner);
  virtual ~GuiPlanner(){};
  void ResetPlanner();
  void WriteGraphs();
  void ReadGraphs();
  //  void DrawGraphs();
  void ReadAnimationFrames();
  void WriteAnimationFrames();
  void ReadPath();
  void WritePath();
  void DrawGraphs();

  friend class MSLPlotWindow;
};

#endif
