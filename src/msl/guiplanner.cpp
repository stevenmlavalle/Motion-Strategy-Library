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

#include "msl/guiplanner.h"

// *********************************************************************
// *********************************************************************
// CLASS:     MSLPlannerWindow
//
// *********************************************************************
// *********************************************************************

// Map
FXDEFMAP(MSLPlannerWindow) MSLPlannerWindowMap[]={
  FXMAPFUNC(SEL_TIMEOUT,FXMainWindow::ID_LAST,MSLPlannerWindow::onCmdTimer),
  FXMAPFUNCS(SEL_COMMAND, 20, 300, MSLPlannerWindow::GeneralHandler)
};

// Object implementation
FXIMPLEMENT(MSLPlannerWindow,FXMainWindow,MSLPlannerWindowMap,ARRAYNUMBER(MSLPlannerWindowMap))

// Make some windows
MSLPlannerWindow::MSLPlannerWindow(GuiPlanner* gp):FXMainWindow(gp,"Motion Strategy Library",NULL,NULL,DECOR_ALL,20,20,500,260) {

  GP = gp;

  // Make the target connections
  Restart();

  // Menubar
  menubar=new FXMenuBar(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X);
  plannermenu = new FXMenuPane(this);
    new FXMenuCommand(plannermenu,"RRT",NULL,this,GID_RRT);
    new FXMenuCommand(plannermenu,"RRTGoalBias",NULL,this,GID_RRTGOALBIAS);
    new FXMenuCommand(plannermenu,"RRTCon",NULL,this,GID_RRTCON);
    new FXMenuCommand(plannermenu,"RRTDual",NULL,this,GID_RRTDUAL);
    new FXMenuCommand(plannermenu,"RRTExtExt",NULL,this,GID_RRTEXTEXT);
    new FXMenuCommand(plannermenu,"RRTExtCon",NULL,this,GID_RRTEXTCON);
    new FXMenuCommand(plannermenu,"RRTConCon",NULL,this,GID_RRTCONCON);
    new FXMenuCommand(plannermenu,"RCRRT",NULL,this,GID_RCRRT);
    new FXMenuCommand(plannermenu,"RCRRTExtExt",NULL,this,GID_RCRRTEXTEXT);
    new FXMenuCommand(plannermenu,"RRTBidirBalanced",NULL,this,GID_RRTBIDIRBALANCED);
    new FXMenuCommand(plannermenu,"PRM",NULL,this,GID_PRM);
    new FXMenuCommand(plannermenu,"FDP",NULL,this,GID_FDP);
    new FXMenuCommand(plannermenu,"FDPStar",NULL,this,GID_FDPSTAR);
    new FXMenuCommand(plannermenu,"FDPBestFirst",NULL,this,GID_FDPBESTFIRST);
    new FXMenuCommand(plannermenu,"FDPBi",NULL,this,GID_FDPBI);
  new FXMenuTitle(menubar,"&Planner",NULL,plannermenu);

  loadmenu=new FXMenuPane(this);
    new FXMenuCommand(loadmenu,"Load Graph(s)",NULL,this,GID_LOAD_GRAPHS,0);
    new FXMenuCommand(loadmenu,"Load Path",NULL,this,GID_LOAD_PATH,0);
    new FXMenuCommand(loadmenu,"Load Animation Frames",NULL,this,GID_LOAD_FRAMES,0);
  new FXMenuTitle(menubar,"&Load",NULL,loadmenu);

  savemenu=new FXMenuPane(this);
    new FXMenuCommand(savemenu,"Save Graph(s)",NULL,this,GID_SAVE_GRAPHS,0);
    new FXMenuCommand(savemenu,"Save Path",NULL,this,GID_SAVE_PATH,0);
    new FXMenuCommand(savemenu,"Save Animation Frames",NULL,this,GID_SAVE_FRAMES,0);
  new FXMenuTitle(menubar,"&Save",NULL,savemenu);

  plotmenu=new FXMenuPane(this);
    new FXMenuCommand(plotmenu,"Draw 2D Graphs",NULL,this,GID_2D_GRAPH,0);
  new FXMenuTitle(menubar,"Plo&t",NULL,plotmenu);

  new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|SEPARATOR_GROOVE|LAYOUT_FILL_X);

  // Buttons
  buttonbar=new FXMenuBar(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X);
  new FXButton(buttonbar,"&Construct",NULL,this,GID_CONSTRUCT,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(buttonbar,"P&lan",NULL,this,GID_PLAN,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(buttonbar,"C&lear",NULL,this,GID_CLEAR_GRAPHS,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(buttonbar,"&Quit",NULL,this,GID_DONE,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);

  new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|SEPARATOR_GROOVE|LAYOUT_FILL_X);

  new FXLabel(this," Planner Controls                                    Animation Controls",
    NULL,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|JUSTIFY_LEFT);

  new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|SEPARATOR_GROOVE|LAYOUT_FILL_X);

  vcrbar=new FXMenuBar(this,LAYOUT_SIDE_BOTTOM|LAYOUT_FILL_X);
  new FXLabel(vcrbar,"Movie Buttons",NULL,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|JUSTIFY_LEFT);
  new FXButton(vcrbar,"Stop",NULL,this,GID_VCR_STOP,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Last",NULL,this,GID_VCR_LAST,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Pause",NULL,this,GID_VCR_PAUSE,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Next",NULL,this,GID_VCR_NEXT,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Slower",NULL,this,GID_VCR_SLOWER,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Play",NULL,this,GID_VCR_PLAY,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Faster",NULL,this,GID_VCR_FASTER,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);
  new FXButton(vcrbar,"Reset",NULL,this,GID_VCR_RESET,FRAME_RAISED|FRAME_THICK|LAYOUT_FILL_X);

  // Arange nicely
  matrix=new FXMatrix(this,5,MATRIX_BY_COLUMNS|LAYOUT_SIDE_TOP|LAYOUT_FILL_X|LAYOUT_FILL_Y);

  new FXLabel(matrix,"PlannerDeltaT",NULL,LAYOUT_CENTER_Y|LAYOUT_CENTER_X|JUSTIFY_RIGHT|LAYOUT_FILL_ROW);
  new FXTextField(matrix,10,&plannerdeltat_target,FXDataTarget::ID_VALUE,TEXTFIELD_REAL|JUSTIFY_RIGHT|LAYOUT_CENTER_Y|LAYOUT_CENTER_X|FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_ROW);
  new FXFrame(matrix,LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW);
  new FXLabel(matrix,"AnimationTimeScale",NULL,LAYOUT_CENTER_Y|LAYOUT_CENTER_X|JUSTIFY_RIGHT|LAYOUT_FILL_ROW);
  FXSlider* fsl = new FXSlider(matrix,&animationtimescale_target,FXDataTarget::ID_VALUE,LAYOUT_CENTER_Y|LAYOUT_FILL_ROW|LAYOUT_FIX_WIDTH,0,0,100);
  fsl->setRange(1,100);

  new FXLabel(matrix,"NumNodes",NULL,LAYOUT_CENTER_Y|LAYOUT_CENTER_X|JUSTIFY_RIGHT|LAYOUT_FILL_ROW);
  new FXTextField(matrix,10,&numnodes_target,FXDataTarget::ID_VALUE,TEXTFIELD_INTEGER|JUSTIFY_RIGHT|LAYOUT_CENTER_Y|LAYOUT_CENTER_X|FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_ROW);
  new FXFrame(matrix,LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW);
  new FXFrame(matrix,LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW);
  //new FXLabel(matrix,"AmbientLight",NULL,LAYOUT_CENTER_Y|LAYOUT_CENTER_X|JUSTIFY_RIGHT|LAYOUT_FILL_ROW);
  new FXTextField(matrix,10,&animationtimescale_target,FXDataTarget::ID_VALUE,TEXTFIELD_INTEGER|JUSTIFY_RIGHT|LAYOUT_CENTER_Y|LAYOUT_CENTER_X|FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_ROW);
  //new FXSlider(matrix,&ambientlight_target,FXDataTarget::ID_VALUE,LAYOUT_CENTER_Y|LAYOUT_FILL_ROW|LAYOUT_FIX_WIDTH,0,0,100);

  new FXLabel(matrix,"DrawIndexX",NULL,LAYOUT_CENTER_Y|LAYOUT_CENTER_X|JUSTIFY_RIGHT|LAYOUT_FILL_ROW);
  new FXTextField(matrix,10,&drawindexx_target,FXDataTarget::ID_VALUE,TEXTFIELD_INTEGER|JUSTIFY_RIGHT|LAYOUT_CENTER_Y|LAYOUT_CENTER_X|FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_ROW);
  //  new FXSpinner(matrix,5,&drawindexx_target,0,SPIN_CYCLIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_CENTER_Y|LAYOUT_FILL_ROW);
  new FXFrame(matrix,LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW);

  new FXToggleButton(matrix,"Bounding Box OFF","Bounding Box ON",NULL,NULL,this,GID_TOGGLE_BOUNDINGBOX,LAYOUT_CENTER_Y|LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW|ICON_BEFORE_TEXT);
  new FXToggleButton(matrix,"Multiple Views OFF","Multiple Views ON",NULL,NULL,this,GID_TOGGLE_MULTIPLEVIEWS,LAYOUT_CENTER_Y|LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW|ICON_BEFORE_TEXT);

  new FXLabel(matrix,"DrawIndexY",NULL,LAYOUT_CENTER_Y|LAYOUT_CENTER_X|JUSTIFY_RIGHT|LAYOUT_FILL_ROW);
  new FXTextField(matrix,10,&drawindexy_target,FXDataTarget::ID_VALUE,TEXTFIELD_INTEGER|JUSTIFY_RIGHT|LAYOUT_CENTER_Y|LAYOUT_CENTER_X|FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_ROW);
  //new FXSpinner(matrix,5,&drawindexy_target,1,SPIN_CYCLIC|FRAME_SUNKEN|FRAME_THICK|LAYOUT_CENTER_Y|LAYOUT_FILL_ROW);
  new FXFrame(matrix,LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW);
  new FXToggleButton(matrix,"Attached Camera OFF","Attached Camera ON",NULL,NULL,this,GID_TOGGLE_ATTACHEDCAMERA,LAYOUT_CENTER_Y|LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW|ICON_BEFORE_TEXT);
  new FXToggleButton(matrix,"Show Path OFF","Show Path ON",NULL,NULL,this,GID_TOGGLE_SHOWPATH,LAYOUT_CENTER_Y|LAYOUT_FILL_COLUMN|LAYOUT_FILL_ROW|ICON_BEFORE_TEXT);

  new FXHorizontalSeparator(this,LAYOUT_SIDE_TOP|SEPARATOR_GROOVE|LAYOUT_FILL_X);

  // Install an accelerator
  //getAccelTable()->addAccel(fxparseaccel("Ctl-Q"),getApp(),MKUINT(-1,SEL_COMMAND));
  }


// Clean up
MSLPlannerWindow::~MSLPlannerWindow(){
  delete loadmenu;
  delete savemenu;
  delete plotmenu;
  delete plannermenu;
  }


// Timer
long MSLPlannerWindow::onCmdTimer(FXObject*,FXSelector,void*) {

  // Reset timer for next time
  //  getApp()->addTimeout(80,this,FXMainWindow::ID_LAST);
  getApp()->addTimeout(this,FXMainWindow::ID_LAST,80);
  return 1;
}



// Start
void MSLPlannerWindow::create(){

  // Create windows
  FXMainWindow::create();

  // Kick off the timer
  //  getApp()->addTimeout(80,this,FXMainWindow::ID_LAST);
  getApp()->addTimeout(this,FXMainWindow::ID_LAST,80);

  // Show
  show(PLACEMENT_SCREEN);
}



void MSLPlannerWindow::Restart() {

  // Make the target connections
  plannerdeltat_target.connect(GP->Pl->PlannerDeltaT);
  numnodes_target.connect(GP->Pl->NumNodes);
  drawindexx_target.connect(GP->DrawIndexX);
  drawindexy_target.connect(GP->DrawIndexY);
  animationtimescale_target.connect(GP->R->AnimationTimeScale);
  ambientlight_target.connect(GP->R->AmbientLight);
}


// Timer
long MSLPlannerWindow::GeneralHandler(FXObject*,FXSelector sel,void*) {
  int i = FXSELID(sel);
  //cout << "General Handler  -- selection " << i << "\n";

  GP->ButtonHandle(i);

  return 1;
}




// *********************************************************************
// *********************************************************************
// CLASS:     MSLPlotWindow
//
// *********************************************************************
// *********************************************************************



// Message Map for the Plot Window class
FXDEFMAP(MSLPlotWindow) MSLPlotWindowMap[]={

  //________Message_Type_______________________________Message_Handler_______
  FXMAPFUNC(SEL_PAINT,MSLPlotWindow::ID_CANVAS, MSLPlotWindow::onPaint),
  FXMAPFUNC(SEL_COMMAND,MSLPlotWindow::ID_PRINT, MSLPlotWindow::onCmdPrint),
};



// Macro for the PlotApp class hierarchy implementation
FXIMPLEMENT(MSLPlotWindow,FXDialogBox,MSLPlotWindowMap,ARRAYNUMBER(MSLPlotWindowMap))



// Construct a MSLPlotWindow
MSLPlotWindow::MSLPlotWindow(MSLPlannerWindow* owner):
  FXDialogBox(owner,"Plot Window",DECOR_ALL,0,0,500,500) {

  Owner = owner;
  GP = owner->GP;
  indexx = GP->DrawIndexX;
  indexy = GP->DrawIndexY;

  if ((indexx >= GP->Pl->P->StateDim)||(indexx < 0))
    indexx = 0;
  if ((indexy >= GP->Pl->P->StateDim)||(indexy < 0))
    indexy = 1;

  char* title = new char[80];
  sprintf(title,"Search Graph(s):   x[%d] vs. x[%d]",indexx,indexy);

  contents=new FXHorizontalFrame(this,LAYOUT_SIDE_TOP|LAYOUT_FILL_X|LAYOUT_FILL_Y,0,0,0,0, 0,0,0,0);

  // LEFT pane to contain the canvas
  canvasFrame=new FXVerticalFrame(contents,FRAME_SUNKEN|LAYOUT_FILL_X|LAYOUT_FILL_Y|LAYOUT_TOP|LAYOUT_LEFT,0,0,0,0,10,10,10,10);

    // Label above the canvas
  new FXLabel(canvasFrame,title,NULL,JUSTIFY_CENTER_X|LAYOUT_FILL_X);

    // Horizontal divider line
  new FXHorizontalSeparator(canvasFrame,SEPARATOR_GROOVE|LAYOUT_FILL_X);

    // Drawing canvas
  canvas=new FXCanvas(canvasFrame,this,ID_CANVAS,FRAME_SUNKEN|FRAME_THICK|LAYOUT_FILL_X|LAYOUT_FILL_Y|LAYOUT_TOP|LAYOUT_LEFT);

  // RIGHT pane for the buttons
  buttonFrame=new FXVerticalFrame(contents,FRAME_SUNKEN|LAYOUT_FILL_Y|LAYOUT_TOP|LAYOUT_LEFT,0,0,0,0,10,10,10,10);

    // Label above the buttons
    new FXLabel(buttonFrame,"Options",NULL,JUSTIFY_CENTER_X|LAYOUT_FILL_X);

    // Horizontal divider line
    new FXHorizontalSeparator(buttonFrame,SEPARATOR_RIDGE|LAYOUT_FILL_X);

    // Button to print
    new FXButton(buttonFrame,"&Print",NULL,this,ID_PRINT,FRAME_THICK|FRAME_RAISED|LAYOUT_FILL_X|LAYOUT_TOP|LAYOUT_LEFT,0,0,0,0,10,10,5,5);

    // Exit button
    new FXButton(buttonFrame,"&Exit",NULL,this,ID_HIDE,FRAME_THICK|FRAME_RAISED|LAYOUT_FILL_X|LAYOUT_TOP|LAYOUT_LEFT,0,0,0,0,10,10,5,5);

}



// Paint the canvas
long MSLPlotWindow::onPaint(FXObject*,FXSelector,void* ptr){
  FXEvent *ev=(FXEvent*)ptr;
  FXDCWindow dc(canvas,ev);
  dc.setForeground(canvas->getBackColor());
  dc.fillRectangle(ev->rect.x,ev->rect.y,ev->rect.w,ev->rect.h);
  drawPage(dc,canvas->getWidth(),canvas->getHeight());
  return 1;
}


long MSLPlotWindow::onCmdPrint(FXObject*,FXSelector,void*){
  FXPrintDialog dlg(this,"Print Graphics");
  FXPrinter printer;
  if(dlg.execute()){
    dlg.getPrinter(printer);
    FXTRACE((100,"Printer = %s\n",printer.name.text()));
    FXDCPrint pdc(getApp());
    if(!pdc.beginPrint(printer)) {
      FXMessageBox::error(this,MBOX_OK,"Printer Error","Unable to print");
      return 1;
    }
    pdc.beginPage(1);
    pdc.setLineWidth(1);
    drawPage(pdc,400,400,100,100);
    pdc.endPage();
    pdc.endPrint();
    }
  return 1;
}



// This is the WYSIWYG routine, it takes a DC and renders
// into it; it does not know if the DC is a printer or screen.
void MSLPlotWindow::drawPage(FXDC& dc,FXint w,FXint h,
			     FXint tx,FXint ty) {
  list<MSLNode*> nlist;
  list<MSLNode*>::iterator ni;
  list<MSLEdge*> elist;
  list<MSLEdge*>::iterator ei;
  MSLVector x1,x2;
  double tranx,trany,scalex,scaley;
  int lw;

  //dc.setForeground(erasecolor);
  //dc.fillRectangle(0,0,w,h);

  //dc.setForeground(forecolor);
  //dc.setBackground(FXRGB(0,0,0));

  //dc.setLineStyle(lineStyle);
  //dc.setLineCap(capStyle);
  //dc.setLineJoin(joinStyle);
  //dc.setFunction(function);

  //dc.setStipple(stipple);
  //dc.setFillStyle(fillStyle);
  //dc.setLineWidth(lineWidthSpinner->getValue());
  lw = dc.getLineWidth();
  dc.setLineWidth(0);

  dc.setForeground(FXRGB(0,0,0));
  dc.drawRectangle(tx,ty,w,h);

  scalex = w/(GP->Pl->P->UpperState[indexx] -
	      GP->Pl->P->LowerState[indexx]);
  scaley = -h/(GP->Pl->P->UpperState[indexy] -
	      GP->Pl->P->LowerState[indexy]);
  tranx = tx-1.0 * scalex * GP->Pl->P->LowerState[indexx];
  trany = ty+h-1.0 * scaley * GP->Pl->P->LowerState[indexy];

  // Show path (if it exists)
  dc.setForeground(FXRGB(200,0,0));
  dc.setLineWidth(2);
  list<MSLVector> path;
  list<MSLVector>::iterator pi;
  path = GP->Pl->Path;
  if (path.size() >= 2) {
    x1 = path.front();
    pi = path.begin();
    pi++;
    while (pi != path.end()) {
      x2 = *pi;
      dc.drawLine(x1[indexx]*scalex+tranx,
		  x1[indexy]*scaley+trany,
		  x2[indexx]*scalex+tranx,
		  x2[indexy]*scaley+trany);
      x1 = x2;
      pi++;
    }
  }

  // Show first tree (if it exists)
  dc.setLineWidth(lw);
  dc.setForeground(FXRGB(0,0,255));
  if (GP->Pl->T) {
    nlist = GP->Pl->T->Nodes();
    forall(ni,nlist) {
      if ((*ni)->Parent()) {  // Make sure it has a parent to connect to!
	x1 = (*ni)->State();
	x2 = (*ni)->Parent()->State();
	dc.drawLine(x1[indexx]*scalex+tranx,
		    x1[indexy]*scaley+trany,
		    x2[indexx]*scalex+tranx,
		    x2[indexy]*scaley+trany);
      }
    }
  }
  // Show second tree (if it exists)
  dc.setForeground(FXRGB(255,0,0));
  if (GP->Pl->T2) {
    nlist = GP->Pl->T2->Nodes();
    forall(ni,nlist) {
      if ((*ni)->Parent()) {  // Make sure it has a parent to connect to!
	x1 = (*ni)->State();
	x2 = (*ni)->Parent()->State();
	dc.drawLine(x1[indexx]*scalex+tranx,
		    x1[indexy]*scaley+trany,
		    x2[indexx]*scalex+tranx,
		    x2[indexy]*scaley+trany);
      }
    }
  }

  // Show roadmap (if it exists)
  dc.setForeground(FXRGB(0,150,0));
  if (GP->Pl->Roadmap) {
    elist = GP->Pl->Roadmap->Edges();
    forall(ei,elist) {
      x1 = (*ei)->Source()->State();
      x2 = (*ei)->Target()->State();
      dc.drawLine(x1[indexx]*scalex+tranx,
		  x1[indexy]*scaley+trany,
		  x2[indexx]*scalex+tranx,
		  x2[indexy]*scaley+trany);
    }
  }

  // Restore original values
  dc.setLineWidth(lw);
}



// *********************************************************************
// *********************************************************************
// CLASS:     GuiPlanner
//
// *********************************************************************
// *********************************************************************

GuiPlanner::GuiPlanner(Render *render, Planner *planner):Gui(render) {
  Pl = planner;

  LineWidth = 1.0;
  PSLineWidth = 1.0;
  DrawIndexX = 0;
  DrawIndexY = 1;

  if (!render)
    cout << "ERROR: Renderer no defined\n";

  FilePath = Pl->P->FilePath;

  CreateMenuWindow();
}



void GuiPlanner::Init() {
  list<MSLVector> tpath;

  // Read the planner type, if exists
  if (is_file(Pl->P->FilePath + "RRT"))
    ButtonHandle(GID_RRT);
  if (is_file(Pl->P->FilePath + "RRTGoalBias"))
    ButtonHandle(GID_RRTGOALBIAS);
  if (is_file(Pl->P->FilePath + "RRTCon"))
    ButtonHandle(GID_RRTCON);
  if (is_file(Pl->P->FilePath + "RRTDual"))
    ButtonHandle(GID_RRTDUAL);
  if (is_file(Pl->P->FilePath + "RRTExtExt"))
    ButtonHandle(GID_RRTEXTEXT);
  if (is_file(Pl->P->FilePath + "RRTExtCon"))
    ButtonHandle(GID_RRTEXTCON);
  if (is_file(Pl->P->FilePath + "RRTConCon"))
    ButtonHandle(GID_RRTCONCON);
  if (is_file(Pl->P->FilePath + "RCRRT"))
    ButtonHandle(GID_RCRRT);
  if (is_file(Pl->P->FilePath + "RCRRTExtExt"))
    ButtonHandle(GID_RCRRTEXTEXT);
  if (is_file(Pl->P->FilePath + "RRTBidirBalanced"))
    ButtonHandle(GID_RRTBIDIRBALANCED);
  if (is_file(Pl->P->FilePath + "PRM"))
    ButtonHandle(GID_PRM);
  if (is_file(Pl->P->FilePath + "FDP"))
    ButtonHandle(GID_FDP);
  if (is_file(Pl->P->FilePath + "FDPStar"))
    ButtonHandle(GID_FDPSTAR);
  if (is_file(Pl->P->FilePath + "FDPBestFirst"))
    ButtonHandle(GID_FDPBESTFIRST);
  if (is_file(Pl->P->FilePath + "FDPBi"))
    ButtonHandle(GID_FDPBI);

  Gui::Init();

  // Set up to animate a single frame initially
  tpath.clear();
  //tpath.push(Pl->P->GoalState);
  tpath.push_back(Pl->P->InitialState);
  R->MakeAnimationFrames(tpath, 1.0);
  R->AnimationActive = true;
}


void GuiPlanner::CreateMenuWindow() {

  // Open display (multiple inheritance from FXApp)
  // Make some dummy arg stuff to make FOX happy...
  char  arg0[] = "MSL";
  char* argv[] = { &arg0[0], NULL };
  int   argc   = (int)(sizeof(argv) / sizeof(argv[0])) - 1;
  init(argc,&argv[0]);

  // Main window
  Window=new MSLPlannerWindow(this);

  // Handle interrupt to save stuff nicely (from FXApp)
  //addSignal(SIGINT,Window,FXApp::ID_QUIT);

  // Create app (from FXApp)
  create();

  // Run (from FXApp)
  //run();

}


void GuiPlanner::ResetPlanner()
{
  R->SetScene(new Scene(Pl->P,FilePath));
  Pl->Reset();
  Window->Restart();
}


void GuiPlanner::ButtonHandle(int b){

  if ((b > GID_RENDER_FIRST) && (b < GID_RENDER_LAST))
    R->ButtonHandle(b);
  else {
    switch (b) {
    case GID_CONSTRUCT: cout << "Construct\n";
      Pl->Construct();
      //U.DrawGraphs();
      break;
    case GID_PLAN: cout << "Plan\n";
      if (Pl->Plan()) {
	cout << "Making Animation Frames\n";
	// Make the animation frames
	if ((Pl->Path.size() >= 2)||
	    (R->FrameList.size() > 0))
	  {
	  R->MakeAnimationFrames(Pl->Path,Pl->TimeList);
	  }
      }
      //U.DrawPath();
      //U.DrawGraphs();
      break;
    case GID_CLEAR_GRAPHS: cout << "Clear Graphs\n";
      ResetPlanner();
      break;
    case GID_2D_GRAPH: cout << "2D Graph Projection\n";
      DrawGraphs();
      break;
    case GID_SAVE_GRAPHS: cout << "Write Graph(s)\n";
      WriteGraphs();
      break;
    case GID_LOAD_GRAPHS: cout << "Read Graph(s)\n";
      ReadGraphs();
      break;
    case GID_SAVE_FRAMES: cout << "Write Animation Frames\n";
      WriteAnimationFrames();
      break;
    case GID_LOAD_FRAMES: cout << "Read Animation Frames\n";
      ReadAnimationFrames();
      break;
    case GID_SAVE_PATH: cout << "Write Path\n";
      WritePath();
      break;
    case GID_LOAD_PATH: cout << "Read Path\n";
      ReadPath();
      break;
    case GID_DONE: cout << "Done\n";
      Finished = true;  // This should trigger leaving (for most renderers)
      break;
    case GID_TOGGLE_SHOWPATH: cout << "Toggle Show Path\n";
      R->ShowPathOn = !R->ShowPathOn;
      break;
    case GID_RRT: cout << "Switch to RRT Planner\n";
      ResetPlanner();
      Pl = new RRT(Pl->P); // Keep the old model, change the planner
      break;
    case GID_RRTGOALBIAS: cout << "Switch to RRTGoalBias Planner\n";
      ResetPlanner();
      Pl = new RRTGoalBias(Pl->P); // Keep the old model, change the planner
      break;
    case GID_RRTCON: cout << "Switch to RRTCon Planner\n";
      ResetPlanner();
      Pl = new RRTCon(Pl->P); // Keep the old model, change the planner
      break;
    case GID_RRTDUAL: cout << "Switch to RRTDual Planner\n";
      ResetPlanner();
      Pl = new RRTDual(Pl->P); // Keep the old model, change the planner
      break;
    case GID_RRTEXTEXT: cout << "Switch to RRTExtExt Planner\n";
      ResetPlanner();
      Pl = new RRTExtExt(Pl->P); // Keep the old model, change the planner
      break;
    case GID_RRTEXTCON: cout << "Switch to RRTExtCon (RRT-Connect) Planner\n";
      ResetPlanner();
      Pl = new RRTExtCon(Pl->P);
      break;
    case GID_RRTCONCON: cout << "Switch to RRTConCon Planner\n";
      ResetPlanner();
      Pl = new RRTConCon(Pl->P);
      break;
    case GID_RCRRT: cout << "Switch to RCRRT Planner\n";
      ResetPlanner();
      Pl = new RCRRT(Pl->P);
      break;
    case GID_RCRRTEXTEXT: cout << "Switch to RCRRTExtExt Planner\n";
      ResetPlanner();
      Pl = new RCRRTExtExt(Pl->P);
      break;
    case GID_RRTBIDIRBALANCED: cout << "Switch to RRTBidirBalanced Planner\n";
      ResetPlanner();
      Pl = new RRTBidirBalanced(Pl->P);
      break;
    case GID_PRM: cout << "Switch to PRM Planner\n";
      ResetPlanner();
      Pl = new PRM(Pl->P);
      break;
    case GID_FDP: cout << "Switch to FDP Planner\n";
      ResetPlanner();
      Pl = new FDP(Pl->P);
      break;
    case GID_FDPSTAR: cout << "Switch to FDPStar Planner\n";
      ResetPlanner();
      Pl = new FDPStar(Pl->P);
      break;
    case GID_FDPBESTFIRST: cout << "Switch to FDPBestFirst Planner\n";
      ResetPlanner();
      Pl = new FDPBestFirst(Pl->P);
      break;
    case GID_FDPBI: cout << "Switch to FDPBi Planner\n";
      ResetPlanner();
      Pl = new FDPBi(Pl->P);
      break;

    default: cout << "Option " << b << " not implemented\n";
      break;
    }
  }
}



void GuiPlanner::HandleEvents() {
  runOneEvent();  // Run a FOX event (came from FXApp parent class)
}



void GuiPlanner::WriteGraphs()
{
  FXFileDialog dialog(Window,"Write Solution Path");
  dialog.setDirectory(("./"+FilePath).c_str());
  dialog.setFilename("graphs");
  if (dialog.execute()) {
    std::ofstream outfile(dialog.getFilename().text());
    if (outfile) {
      Pl->WriteGraphs(outfile);
      outfile.close();
    }
  }
}



void GuiPlanner::ReadGraphs()
{
  FXFileDialog dialog(Window,"Read Animation Frames");
  dialog.setDirectory(("./"+FilePath).c_str());
  dialog.setFilename("graphs");
  if (dialog.execute()) {
    std::ifstream infile(dialog.getFilename().text());
    if (infile) {
      Pl->ReadGraphs(infile);
      infile.close();
    }
  }
}



void GuiPlanner::WriteAnimationFrames()
{
  FXFileDialog dialog(Window,"Write Animation Frames");
  dialog.setDirectory(("./"+FilePath).c_str());
  dialog.setFilename("frames");
  if (dialog.execute()) {
    std::ofstream outfile(dialog.getFilename().text());
    if (outfile) {
      outfile << R->FrameList;
      outfile.close();
    }
  }
}


void GuiPlanner::ReadAnimationFrames()
{
  FXFileDialog dialog(Window,"Read Animation Frames");
  dialog.setDirectory(("./"+FilePath).c_str());
  dialog.setFilename("frames");
  if (dialog.execute()) {
    std::ifstream infile(dialog.getFilename().text());
    if (infile) {
      infile >> R->FrameList;
      infile.close();
    }
  }

  R->AnimationActive = true;
}


void GuiPlanner::WritePath()
{
  FXFileDialog dialog(Window,"Write Solution Path");
  dialog.setDirectory(("./"+FilePath).c_str());
  dialog.setFilename("path");
  if (dialog.execute()) {
    std::ofstream outfile(dialog.getFilename().text());
    if (outfile) {
      outfile << Pl->Path;
      outfile.close();
    }
  }
}



void GuiPlanner::ReadPath()
{
  FXFileDialog dialog(Window,"Read Animation Frames");
  dialog.setDirectory(("./"+FilePath).c_str());
  dialog.setFilename("path");
  if (dialog.execute()) {
    std::ifstream infile(dialog.getFilename().text());
    if (infile) {
      infile >> Pl->Path;
      infile.close();
    }
  }

  R->AnimationActive = true;
  R->MakeAnimationFrames(Pl->Path,Pl->TimeList);
}


void GuiPlanner::DrawGraphs()
{
  MSLPlotWindow *pw = new MSLPlotWindow(Window);

  //pw->execute();  This one is modal
  pw->create();
  pw->show();

  cout << "Draw Graphs\n";
}
