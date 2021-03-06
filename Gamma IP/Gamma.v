`timescale 1ns / 1ps

module Gamma_Multi_Upd_Aff_Subs_Multi_Shapes_232_448(
    input clock,
    input ready_Params,
    input ready_Grad,
    input ready_Coord,
    input [31:0] num_of_subsets,
    input [31:0] optimization_method_,
    input [31:0] correlation_routine_,
    input [31:0] num_pixels,
    input [31:0] cx_, 
    input [31:0] cy_,
    input [31:0] base_address,
    input [31:0] num_pxl_Int,
    input [31:0] num_pxl_FP,
    input [31:0] dout_ref_ints,
    input [31:0] dout_def_ints,
    input [31:0] dout_grad_x_,
    input [31:0] dout_grad_y_,
    input [31:0] x,
    input [31:0] y,
    output reg sub_cood_ea_x,
    output reg sub_cood_ea_y,
    output reg sub_cood_we_x,
    output reg sub_cood_we_y,
    output reg [15:0] sub_cood_addr_x,
    output reg [15:0] sub_cood_addr_y,
    output reg [16:0] addr_ref_ints,
    output reg [16:0] addr_def_ints,
    output reg [16:0] addr_grad_x_,
    output reg [16:0] addr_grad_y_,
    output reg gam_done,
    output reg [31:0] disp_x,
    output reg [31:0] disp_y,
    output reg [31:0] disp_z,
    output reg [31:0] Process_Done,
    input [31:0] new_frame,
    output reg gam_new_subset = 1'b0,
    input gam_interface_done,
    output reg [31:0] gid = 32'b0,
    output reg results_done,
    output reg [127:0] gam_idle_counter = 128'b0,
    output reg gam_busy = 1'b0
);

parameter Num_Subs = 14; //# of subsets
parameter N = 5; //Upd
reg [1:0] clk_counter_a = 2'b0;
reg [1:0] clk_counter_b = 2'b0;
reg [1:0] clk_counter_scx = 2'b0;
reg [1:0] clk_counter_scy = 2'b0;
//reg [95:0] residuals_out;  
reg [191:0] residuals_out;   //aff
// global field values
reg [31:0] SUBSET_DISPLACEMENT_X_FS [0:Num_Subs]; //Upd
reg [31:0] SUBSET_DISPLACEMENT_Y_FS [0:Num_Subs]; //Upd
reg [31:0] ROTATION_Z_FS [0:Num_Subs]; //Upd
reg [31:0] NORMAL_STRETCH_XX_FS [0:Num_Subs]; //Upd
reg [31:0] NORMAL_STRETCH_YY_FS [0:Num_Subs]; //Upd
reg [31:0] SHEAR_STRETCH_XY_FS [0:Num_Subs]; //Upd
reg [31:0] subset_gid;
//reg [31:0] gid = 32'b0; //global id of each subset
reg [31:0] prev_u, prev_v, prev_t;
reg [31:0] initialize_out;
reg [31:0] GmF;
reg [31:0] meanF, meanG;
reg [31:0] sumF;
reg [31:0] sumG;
reg [31:0] sfG, sfF, mfgr, mfrr, Hij, mfhh10, mfhh01, det_h, mfhh10_mfhh01, norm_H, cond_2x2;
reg [31:0] mfdd, dfdd, norm_Hi, mfhq;
reg [31:0] q [0:N];
reg [31:0] H [0:N][0:N];
reg [31:0] def_update [0:N];
reg [31:0] parameters_ [0:N];
reg [31:0] def_old [0:N]; //Upd
reg [31:0] old_t, old_v, old_u;
reg [31:0] def_imgs_ [0:N];
reg [31:0] prev_imgs_ [0:N];
reg [31:0] accumulated_disp = 32'b0;
reg [9:0] state = 'b0;
reg [9:0] temp_state, initial_guess_temp_state, initialize_temp_state, mean_temp_state, residuals_temp_state;
reg [9:0] map_to_u_v_theta_temp_state, insert_motion_temp_state, map_temp_state, initial_guess_4_temp_state;
reg [9:0] interpolate_bilinear_temp_state, interpolate_grad_x_bilinear_temp_state, interpolate_grad_y_bilinear_temp_state;
reg [9:0] sin_temp_state, cos_temp_state, asin_temp_state, acos_temp_state;
reg [9:0] Float_to_Int_temp_state, test_for_convergence_temp_state, update_temp_state;
reg [9:0] map_aff_temp_state, muvt_aff_temp_state, residuals_aff_temp_state, insert_motion_aff_temp_state, test_for_convergence_aff_temp_state;
reg [9:0] save_fields_temp_state;
// The current version does not support this feature (pixel deactivation)
//reg [8:0] subset_is_active = 9'b111111111; //# subset pixels
reg [1:0] interp = 2'b00;
reg use_incremental_formulation_ = 1'b1;
reg converged = 1'b1;
reg MAX_ITERATIONS_REACHED;
reg Correlation_Done;
reg output_deformed_subset_images_;
reg init_status;
reg DICE_ENABLE_GLOBAL = 1'b0;
reg obj_vec_empty;
reg target = 1'b0;
reg corr_status;
reg [31:0] subset_global_id [0:Num_Subs]; //Num_Subs = # subsets //Upd
reg initial_guess_4_out;
reg insert_motion_out;
reg closest_triad_out;
reg [31:0] QUAD_F_FS, QUAD_L_FS, QUAD_A_FS, QUAD_B_FS, QUAD_G_FS, QUAD_H_FS;
reg [31:0] QUAD_C_FS, QUAD_D_FS, QUAD_E_FS;
reg [31:0] QUAD_I_FS, QUAD_J_FS, QUAD_K_FS;
// The current version does not support this feature (pixel deactivation)
//reg [8:0] is_active_ = 9'b111111111; //# subset pixels
reg [31:0] neighbors_ [0:1];
reg [31:0] intensities_ [0:3449]; //assuminh max suset_size is 3450
reg [31:0] offset_x = 32'b0;
reg [31:0] offset_y = 32'b0;
reg [31:0] width = 32'b01000011111000000000000000000000; //448
reg [31:0] height = 32'b01000011011010000000000000000000; //232
reg has_gradients_ = 1'b1;
// for 232*448
reg [31:0] grad_x [0:103935];
reg [31:0] grad_y [0:103935];
reg [31:0] guess_t, guess_v, guess_u;
reg [31:0] temp_H_i, temp_q_i;
//integer num_pixels_ = 9; //for subset
integer state_counter = 0;
integer subset_index = 0;
integer num_iterations;
integer max_solve_its = 25;
integer solve_it = 0;
integer index = 0;
integer i = 0;
integer j = 0;
integer k = 0;
integer l = 0;
integer p = 0;
integer prev_imgs_size = 2;
integer num_neighbors_ = 1;
integer counter_int = 0;
integer counter_grad = 0;
integer factor = 32;

//////////////////////////////// for new functions ///////////////////////////
    reg [31:0] a; 
    reg [31:0] b;
    reg [31:0] a_tmp; 
    reg s1, s2, sign, sr;
    reg [7:0] e1, e2, exy, diff, exponent, e_sum;
    reg [23:0]m1,m2,mx,my;
    reg [24:0]mxy,mxy2;
    reg [31:0] Adder_Float, Subtractor_Float, Multiplier_Float, Divider_Float, SQRT;
    reg [31:0] cos, sin, asin, acos;
    reg [31:0] result, tmp_result;
    reg done;
    reg [47:0] prod, product;
    reg [22:0] x_mantissa;
    reg [7:0] x_exponent;
    reg x_sign;
    reg [22:0] y_mantissa;
    reg [7:0] y_exponent;
    reg y_sign;
    reg [22:0] z_mantissa;
    reg [7:0] z_exponent;
    reg z_sign;
    reg [24:0] partial_remainder;
    reg [24:0] tmp_remainder;
    reg [8:0] exponent_aux; 
    reg multiplier_done;
    reg [25:0] ix;
    reg [25:0] aa;
    reg [51:0] aa_sqrt;
    reg [25:0] biti;
    reg [51:0] r;
    reg [52:0] rt;
    reg [31:0] t1, t2, t3;
    reg cos_done, arith_done, sin_done, acos_done, asin_done;
    integer i_for;
    //for insert_motion
    reg [31:0] im_u, im_v, im_theta;
    reg insert_motion;
	// for insert_motion_aff
	reg [31:0] imaff_u, imaff_v, imaff_theta;
    //for gamma
    reg [31:0] temp_gamma, value, gamma_t1, gamma_t2, gamma_;
    reg [9:0] gamma_temp_state;
    reg [31:0] gamma_mean_def, gamma_mean_sum_def, gamma_mean_sum_ref, gamma_mean_ref;
    integer gamma_i; 
    //for map_to_u_v_theta
    reg [31:0] muvt_cx, muvt_cy;
    reg [31:0] muvt_cxp, muvt_cyp, muvt_cxy, muvt_cx_5, muvt_ax, muvt_ay, muvt_ax_s, muvt_ay_s;
    reg [31:0] muvt_mag_a, muvt_bx, muvt_by, muvt_mag_b, muvt_a_dot_b_over_mags, muvt_rxp, muvt_ryp;
    reg [95:0] map_to_u_v_theta;
	// for map_to_u_v_theta_aff
	reg [31:0] muvt_aff_cx, muvt_aff_cy;
	reg [95:0] map_to_u_v_theta_aff;
    //for map
    reg [31:0] map_x, map_y, map_cx, map_cy, map_dx, map_dy, map_t1, map_t2;
    reg [63:0] map;
	//for map Affine
	reg [31:0] map_aff_x, map_aff_y, map_aff_cx, map_aff_cy, map_aff_out_x, map_aff_out_y;
	reg [31:0] map_aff_dx, map_aff_dy, map_aff_Dx, map_aff_Dy;
	reg [31:0] map_aff_cost, map_aff_sint;
	reg [31:0] map_aff_t1, map_aff_t2, map_aff_t3;
	reg [63:0] map_aff;
    //for mean
    reg mean_ref_def;
    reg [31:0] mean_temp_sum, mean_subset_size_in_ieee;
    reg [63:0] mean;
    integer i_mean;
    //for initial_guess
    reg [31:0] ig_gid, ig_prev_u, ig_prev_v, ig_prev_t;
    reg ig_def_img, initial_guess;
    //initial_guess_4
    reg ig4_def_image, ig4_neigh_id;
    reg [31:0] ig4_u, ig4_v, ig4_t, ig4_id, ig4_dist_, ig4_gamma;
    reg [31:0] ig4_best_u, ig4_best_v, ig4_best_t, ig4_best_gamma;
    integer ig4_neigh;
    //for initialize
    reg initialize_target; //0 = ref, 1 = def
    reg [1:0] initialize_interp; //0 = BILINEAR
    reg [31:0] initialize_px, initialize_py, initialize_t1, initialize_t2, initialize_ox, initialize_oy;
    reg [31:0] initialize_mapped_x, initialize_mapped_y;
    reg [63:0] initialize_out_map;
    reg [31:0] initialize;
    integer initialize_i = 0;
    //for residuals
    reg [31:0] residuals_x, residuals_y, residuals_cx, residuals_cy, residuals_gx, residuals_gy;
    reg residuals_use_ref_grads;
    reg [31:0] residuals_dx, residuals_dy, residuals_Gx, residuals_Gy;
    reg [31:0] residuals_u, residuals_v, residuals_theta, residuals_cosTheta, residuals_sinTheta, residuals_t1, residuals_t2, residuals_t3;
    reg [383:0] residuals;	
	// for residuals_aff
	reg [31:0] residuals_aff_x, residuals_aff_y, residuals_aff_cx, residuals_aff_cy, residuals_aff_gx, residuals_aff_gy;
	reg residuals_aff_use_ref_grads;
	reg [31:0] residuals_aff_dx, residuals_aff_dy, residuals_aff_Dx, residuals_aff_Dy;
	reg [31:0] residuals_aff_delTheta, residuals_aff_delEx, residuals_aff_delEy, residuals_aff_delGxy, residuals_aff_Gx, residuals_aff_Gy;
	reg [31:0] residuals_aff_theta, residuals_aff_dudx, residuals_aff_dvdy, residuals_aff_gxy, residuals_aff_cosTheta, residuals_aff_sinTheta;
	reg [31:0] residulas_t1, residulas_t2, residulas_t3, residulas_t4, residulas_t5, residulas_t6;
	reg [191:0] residuals_aff; 
    //for interpolate_bilinear
    reg [31:0] ib_local_x, ib_local_y;
    reg [31:0] ib_width_ = 32'b01000011111000000000000000000000; //Ver
    reg [31:0] ib_height_ = 32'b01000011011010000000000000000000; //Ver
    reg [31:0] ib_x1, ib_x2, ib_y1, ib_y2;
    reg [31:0] ib_t1, ib_t2, ib_t3, ib_t4, ib_t5, ib_t6;
    reg [31:0] interpolate_bilinear;
    //for interpolate_grad_x_bilinear;
    reg [31:0] igxb_local_x, igxb_local_y;
    reg [31:0] igxb_width_in = 32'b01000011111000000000000000000000; //Ver
    reg [31:0] igxb_height_in = 32'b01000011011010000000000000000000; //Ver
    reg [31:0] igxb_x1, igxb_x2, igxb_y1, igxb_y2;
    reg [31:0] igxb_t1, igxb_t2, igxb_t3, igxb_t4, igxb_t5, igxb_t6;
    reg [31:0] interpolate_grad_x_bilinear;
    //for interpolate_grad_y_bilinear;
    reg [31:0] igyb_local_x, igyb_local_y;
    reg [31:0] igyb_width_in = 32'b0;
    reg [31:0] igyb_height_in = 32'b0;
    reg [31:0] igyb_x1, igyb_x2, igyb_y1, igyb_y2;
    reg [31:0] igyb_t1, igyb_t2, igyb_t3, igyb_t4, igyb_t5, igyb_t6;
    reg [31:0] interpolate_grad_y_bilinear;
    //for Float_to_Int;
    reg [31:0] fti_input_a, fti_a_m, fti_a, fti_z;
    reg [8:0] fti_a_e;
    reg fti_a_s;        
    reg [31:0] fti_s_output_z;
    reg [31:0] Float_to_Int;
    integer fti_i;
	//for test_for_convergence
	reg [31:0] fast_solver_tolerance = 32'b00111000110100011011011100010111; //10^-4 //Upd
	// for test_for_convergence_aff  
    //for multiple subsets
    reg [31:0] subset_range_selection;
    
always @(posedge clock)
begin
    case (state)
    9'b000000000:
    begin 
        if(ready_Params == 1'b1 && ready_Coord == 1'b1 && ready_Grad == 1'b1)
        begin
            //gam_done = 1'b0;
            // Setting the BRAM subset_coordinates control signals
            sub_cood_ea_x = 1'b1;
            sub_cood_ea_y = 1'b1;
            sub_cood_we_x = 1'b0;
            sub_cood_we_y = 1'b0;
            gam_busy = 1'b1;
            state = 'b101110010;       
        end
        else
        begin
            gam_done = 1'b0;
            Process_Done = 32'b0;
            state = 'b000000000;
        end
    end
    'b101110010:
    begin
        results_done = 1'b0;
        if(gid < num_of_subsets)
        begin
            gam_new_subset = 1'b1;
            if(gam_interface_done == 1'b0)
            begin
                state = 'b101110010;
            end
            else //ready to start the subset
            begin
                gam_new_subset = 1'b0;
                subset_range_selection = gid * 32 + 31;
                if(use_incremental_formulation_ == 1'b1)
                begin
                    SUBSET_DISPLACEMENT_X_FS[gid] = 32'b0;
                    SUBSET_DISPLACEMENT_Y_FS[gid] = 32'b0; 
                    ROTATION_Z_FS[gid] = 32'b0;
                    NORMAL_STRETCH_XX_FS [gid] = 32'b0;
                    NORMAL_STRETCH_YY_FS [gid] = 32'b0;
                    SHEAR_STRETCH_XY_FS [gid] = 32'b0;
                    parameters_[0] = 32'b0;
                    parameters_[1] = 32'b0;
                    parameters_[2] = 32'b0;
                    parameters_[3] = 32'b0;
                    parameters_[4] = 32'b0;
                    parameters_[5] = 32'b0;
                    H[0][0] = 32'b0;
                    H[0][1] = 32'b0;
                    H[0][2] = 32'b0;
                    H[0][3] = 32'b0;
                    H[0][4] = 32'b0;
                    H[0][5] = 32'b0;                   
                    H[1][0] = 32'b0;
                    H[1][1] = 32'b0;
                    H[1][2] = 32'b0;
                    H[1][3] = 32'b0;
                    H[1][4] = 32'b0;
                    H[1][5] = 32'b0;                   
                    H[2][0] = 32'b0;
                    H[2][1] = 32'b0;
                    H[2][2] = 32'b0;
                    H[2][3] = 32'b0;
                    H[2][4] = 32'b0;
                    H[2][5] = 32'b0;                   
                    H[3][0] = 32'b0;
                    H[3][1] = 32'b0;
                    H[3][2] = 32'b0;
                    H[3][3] = 32'b0;
                    H[3][4] = 32'b0;
                    H[3][5] = 32'b0;                 
                    H[4][0] = 32'b0;
                    H[4][1] = 32'b0;
                    H[4][2] = 32'b0;
                    H[4][3] = 32'b0;
                    H[4][4] = 32'b0;
                    H[4][5] = 32'b0;                 
                    H[5][0] = 32'b0;
                    H[5][1] = 32'b0;
                    H[5][2] = 32'b0;
                    H[5][3] = 32'b0;
                    H[5][4] = 32'b0;
                    H[5][5] = 32'b0;
                end
                state = 9'b000000001;
            end
        end
        else
        begin
            gam_busy = 1'b0;
            state = 'b000110110; //Process_Done
        end
    end
    9'b000000001:
    begin
        if(DICE_ENABLE_GLOBAL == 1'b1)
        begin
            state = 9'b000101111;
        end
        if(correlation_routine_ == 32'b0) //TRACKING_ROUTINE
        begin
            if(Num_Subs == 0) //# subsets
            begin
                obj_vec_empty = 1'b1;
            end
            else
            begin
                obj_vec_empty = 1'b0;
            end
                
            if(obj_vec_empty == 1'b1)
            begin
                subset_index = 0;
                state = 9'b000000010;
            end
            else
            begin
                state = 9'b000000011;
            end
        end
        else if(correlation_routine_ == 32'b1)
        begin
            gam_done = 1'b0;
        end
        else
        begin
           state = 9'b000110001; 
        end   
    end
    'b000000010:
    begin
        if(subset_index < num_of_subsets)
        begin
            //subset_gid = subset_global_id(subset_index);
            subset_gid = subset_global_id_function(subset_index);
            //obj_vec_.push_back(Teuchos::rcp(new Objective_ZNSSD(this,subset_gid))); an array ****
            // set the sub_image id for each subset: **** find a way are defined for global make it for image
            subset_index = subset_index + 1;
            state = 9'b000000010;
        end
        else
        begin
            state = 9'b000000011;
        end
    end
    'b000000011:
    begin
        //prepare_optimization_initializers() **** NOT DONE does pathfile
        // execute the subsets in order
        subset_index = 0;
        state = 9'b000000100;
    end
    9'b000000100:
    begin
        if(subset_index < num_of_subsets)
        begin
            //check_for_blocking_subsets(subset_gid);
            //generic_correlation_routine(subset_lid); ****
            state = 9'b000000101; 
        end
        else
        begin
            state = 9'b000101111; //if(output_deformed_subset_images_) 
        end
    end
    'b000000101:
    begin
        corr_status = 1'b0; //CORRELATION_FAILED;
        num_iterations =  -1;
        //init_status = initial_guess(gid); //subset_gid
        ig_gid = gid;
        state = 9'b11001000; //initial_guess
        initial_guess_temp_state = 9'b000000110;
    end
    9'b000000110:
    begin
        init_status = initial_guess;
        prev_u = SUBSET_DISPLACEMENT_X_FS[gid];
        prev_v = SUBSET_DISPLACEMENT_Y_FS[gid];
        prev_t = ROTATION_Z_FS[gid];
        // perform the correlation
        if(optimization_method_ == 32'b1)
        begin
            //computeUpdateRobust()
            gam_done = 1'b0;
            state = 'b000110101; //
        end
        else if(optimization_method_ == 32'b0)
        begin
            //computeUpdateFast()
            gam_done = 1'b0; 
            solve_it = 0;
            state = 'b000000111; //11 
        end
    end
    'b000000111:
    begin
        state_counter = state_counter + 1;
        if (solve_it < max_solve_its)
        begin
            state = 'b000001000; //12
        end
        else
        begin
            state = 'b000101111;
        end
    end
    'b000001000:
    begin
        //initialize_out = initialize(target, interp);
        initialize_target = target;
        initialize_interp = interp;
        state = 'b11010011; //initialize
        initialize_temp_state = 'b000001001;
    end
    'b000001001:
    begin
        initialize_out = initialize;
        GmF = 32'b0;
        index = 0;
        gam_done = 1'b0;
        state = 'b000001010;
    end
    'b000001010:
    begin
        //{meanG, sumG} = mean(1'b1);
        mean_ref_def = 1'b1;
        state = 'b11000010; //mean
        mean_temp_state = 'b000001011;
    end
    'b000001011:
    begin
        sumG = mean[31:0];
        meanG = mean[63:32];
        //{meanF, sumF} = mean(1'b0); //ref
        mean_ref_def = 1'b0;
        state = 'b11000010; //mean
        mean_temp_state = 'b000001100;
    end
    'b000001100:
    begin
        sumF = mean[31:0];
        meanF = mean[63:32];
        state = 'b1110100101;
    end
    'b1110100101:
    begin
        if(index < num_pxl_Int) //num_pixels
        begin
            //sfG = Subtractor_Float(def_intensities_[index*32 + 31 -:32], meanG); //indexing changed
            if(clk_counter_b == 2'b10)
            begin
                a = dout_def_ints;
                clk_counter_b =  2'b00;
                b = meanG;
                state = 7'b1000101; //Subtractor
                temp_state = 'b000001101;                 
            end
            else
            begin
                addr_def_ints = index;
                clk_counter_b = clk_counter_b + 1;
                state = 'b1110100101; //'b000001100;
            end
        end
        else
        begin
            state_counter = state_counter + 1;
            state = 'b000011010;
        end
    end
    'b000001101:
    begin
        sfG = Subtractor_Float;
        //sfF = Subtractor_Float(ref_intensities_[index*32 + 31 -:32], meanF); //indexing changed        
        if(clk_counter_a == 2'b10)
        begin
            a = dout_ref_ints;
            clk_counter_a =  2'b00;
            b = meanF;
            state = 7'b1000101; //Subtractor
            temp_state = 'b000001110;                    
        end
        else
        begin
            addr_ref_ints = index;
            clk_counter_a = clk_counter_a + 1;
            state = 'b000001101;
        end
    end
    'b000001110:
    begin
        sfF = Subtractor_Float;
        //GmF = Subtractor_Float(sfG, sfF);
        a = sfG;
        b = sfF;
        state = 7'b1000101; //Subtractor
        temp_state = 'b000001111;
    end
    'b000001111:
    begin
        GmF = Subtractor_Float;
        i = 0;
        state_counter = state_counter + 1; 
        state = 'b000010000;
    end
    'b000010000:
    begin
        if(i <= N)
        begin
           //residuals[i] = 0.0;
           state_counter = state_counter + 1; 
           //residuals_out[i*32+31-:32] = 32'b0;
		   //aff
		   residuals_out[i*32+31-:32] = 32'b0;
           //q[i] = 32'b0; in the C code this is done at the end of each iteration
           i = i + 1;
           state = 'b000010000;
        end
        else
        begin
            state_counter = state_counter + 1; 
            state = 'b101110000; //15
        end
    end
    'b101110000:
    begin
        //residuals_out = residuals(x[index*32 + 31-:32], y[index*32 + 31-:32], cx_, cy_, grad_x_[index*32+31-:32],grad_y_[index*32+31-:32], use_ref_grads); //indexing changed
        // wait two clock cycles 
        //residuals_x = x[index*32 + 31-:32];
        //residuals_y = y[index*32 + 31-:32];
        if(clk_counter_scx == 2'b10)
        begin
            //residuals_x = x;
			//aff
			residuals_aff_x = x;
            clk_counter_scx =  2'b00;
            clk_counter_scy = 2'b01;
            state = 'b101110000;                       
        end
        else if(clk_counter_scy == 2'b00)
        begin
            sub_cood_addr_x = base_address + index;
            clk_counter_scx = clk_counter_scx + 1;
            state = 'b101110000;
        end
        else if(clk_counter_scy == 2'b11)
        begin
            //residuals_y = y;
			//aff
			residuals_aff_y = y;
            clk_counter_scy =  2'b00;
            //go to the next state
            state = 'b000010001;                 
        end
        else
        begin
            sub_cood_addr_y = base_address + index;
            clk_counter_scy = clk_counter_scy + 1;
            state = 'b101110000;
        end
     end   
    'b000010001:
    begin
        state_counter = state_counter + 1;
        if(clk_counter_a == 2'b10)
        begin
            //residuals_gx = dout_grad_x_;
			//aff
			residuals_aff_gx = dout_grad_x_;
            clk_counter_a =  2'b00;
            clk_counter_b = 2'b01;
            state = 'b000010001;                       
        end
        else if(clk_counter_b == 2'b00)
        begin
            addr_grad_x_ =  index;
            clk_counter_a = clk_counter_a + 1;
            state = 'b000010001;
        end
        else if(clk_counter_b == 2'b11)
        begin
            //residuals_gy = dout_grad_y_;
			//aff
			residuals_aff_gy = dout_grad_y_;
            clk_counter_b =  2'b00;
            //residuals_cx = cx_;
            //residuals_cy = cy_;
			//residuals_use_ref_grads = 1'b1; 
            //state = 'b11100111; //residuals;
            //residuals_temp_state = 'b000010010; 
			//aff
			residuals_aff_cx = cx_;
			residuals_aff_cy = cy_;
            residuals_aff_use_ref_grads = 1'b1;
			state = 'b1110010000; //residuals_aff
			residuals_aff_temp_state = 'b000010010;
        end
        else
        begin
            addr_grad_y_ = index;
            clk_counter_b = clk_counter_b + 1;
            state = 'b000010001;
        end           
    end
    'b000010010:
    begin
        //residuals_out = residuals;
		//aff
		residuals_out = residuals_aff;
        i = 32'b0;
        state = 'b000010011;
    end
    'b000010011:
    begin
        if(i <= N)
        begin
            state_counter = state_counter + 1; 
            //q[i] += GmF*residuals[i];
            //mfgr = Multiplier_Float(GmF, residuals_out[i*32+31-:32]);
            a = GmF;
            b = residuals_out[i*32+31-:32];
            state = 7'b1000110; //Multiplier
            temp_state = 'b000010100;
        end
        else
        begin
            state_counter = state_counter + 1;
            state = 'b000011001;
        end
    end
    'b000010100:
    begin
        mfgr = Multiplier_Float;
        j = 0;
        //q[i] = Adder_Float(q[i], mfgr);
        a = q[i];
        b = mfgr;
        state = 7'b1000000; //Adder
        temp_state = 'b000010101; //17
    end
    'b000010101:
    begin
        temp_q_i = Adder_Float;       
        state = 'b000010110;
    end
    'b000010110:
    begin
        q[i] = temp_q_i;
        if(j <= N)
        begin
            state_counter = state_counter + 1; 
            //mfrr = Multiplier_Float(residuals_out[i*32+31-:32], residuals_out[i*32+31-:32]);
            a = residuals_out[i*32+31-:32];
            b = residuals_out[i*32+31-:32];
            //H[i][j] = 32'b0;
            state = 7'b1000110; //Multiplier
            temp_state = 'b000010111;
        end
        else
        begin
            state_counter = state_counter + 1; 
            i = i + 1;
            state = 'b000010011; 
        end
    end
    'b000010111:
    begin
        mfrr = Multiplier_Float;
        //H[i][j] = Adder_Float(H[i][j] , mfrr);
        a = H[i][j];
        b = mfrr;
        state = 7'b1000000; //Adder
        temp_state = 'b000011000;
    end
    'b000011000:
    begin
        temp_H_i = Adder_Float;
        H[i][j] = temp_H_i;
        j = j + 1;
        state = 'b000010110;
    end
    'b000011001:
    begin
        state_counter = state_counter + 1;      
        index = index + 1;
        state = 'b1110100101;
    end
    'b000011010:
    begin
        state_counter = state_counter + 1;
        //mfhh10 = Multiplier_Float(H[1][0], H[0][1]);
        a = H[1][0];
        b = H[0][1];
        state = 7'b1000110; //Multiplier
        temp_state = 'b000011011;
    end
    'b000011011:
    begin
        mfhh10 = Multiplier_Float;
        //mfhh01 = Multiplier_Float(H[0][0], H[1][1]);
        a = H[0][0];
        b = H[1][1];
        state = 7'b1000110; //Multiplier
        temp_state = 'b000011100;
    end
    'b000011100:
    begin
        mfhh01 = Multiplier_Float;
        //det_h = Subtractor_Float(mfhh10, mfhh01);
        a = mfhh10;
        b = mfhh01;
        state = 7'b1000101; //Subtractor
        temp_state = 'b000011101;
    end
    'b000011101:
    begin
        det_h = Subtractor_Float;
        //mfhh10_mfhh01 = Adder_Float(mfhh10, mfhh01);
        a = mfhh10;
        b = mfhh01;
        state = 'b1000000; //Adder 
        temp_state = 'b000011110;
    end
    'b000011110:
    begin
        mfhh10_mfhh01 = Adder_Float;
        //norm_H = SQRT(mfhh10_mfhh01);
        a = mfhh10_mfhh01;
        state = 7'b1010011; //SQRT
        temp_state = 'b000011111;
    end
    'b000011111:
    begin
        norm_H = SQRT;
        cond_2x2 = 32'b10111111100000000000000000000000; //-1
        if(det_h != 0)
        begin
            //mfdd = Multiplier_Float(det_h, det_h);
            a = det_h;
            b = det_h;
            state = 7'b1000110; //Multiplier
            temp_state = 'b000100000;
        end
        else
        begin
            state = 'b000100110;
        end
    end
    'b000100000:
    begin
        mfdd = Multiplier_Float;
        //dfdd = Divider_Float(32'b00111111100000000000000000000000,mfdd);
        a = 32'b00111111100000000000000000000000;
        b = mfdd;
        state = 7'b1001100; //Divider
        temp_state = 'b000100010;
    end
    'b000100010:
    begin
        dfdd = Divider_Float;
        //norm_Hi = Multiplier_Float(dfdd, mfhh10_mfhh01);
        a = dfdd;
        b = mfhh10_mfhh01;
        state = 7'b1000110; //Multiplier
        temp_state = 'b000100011;
    end
    'b000100011:
    begin
        norm_Hi = Multiplier_Float;
        //norm_Hi = SQRT(norm_Hi);
        a = norm_Hi;
        state = 7'b1010011; //SQRT
        temp_state = 'b000100100;
    end
    'b000100100:
    begin
        norm_Hi = SQRT;
        //cond_2x2 = Multiplier_Float(norm_H, norm_Hi);
        a = norm_H;
        b = norm_Hi;
        state = 7'b1000110; //Multiplier
        temp_state = 'b000100101;
    end
    'b000100101:
    begin
        cond_2x2 = Multiplier_Float;
        state = 'b000100110;
    end
    'b000100110:
    begin
        i = 0;
        state = 'b000100111;
    end
    'b000100111:
    begin
        state_counter = state_counter + 1; 
        if(i <= N)
        begin
            // save off last step
            //def_old[i] = (*shape_function)(i); ****
			def_old[i] = residuals_out[i*32+31-:32]; //Upd
            i = i + 1; 
            state = 'b000100111;
        end
        else
        begin
            i = 0;
            state = 'b000101000;
        end
    end
    'b000101000:
    begin
        state_counter = state_counter + 1; 
        if(i <= N)
        begin
            def_update[i] = 32'b0; //Upd
            i = i + 1;
            state = 'b000101000;
        end
        else
        begin
            i = 0;
            j = 0;
            state = 'b000101001;
        end
    end
    'b000101001:
    begin
        state_counter = state_counter + 1; 
        if(i <= N)
        begin
            if(j <= N)
            begin
                //mfhq = Multiplier_Float(H[i][j], q[j]);
                a = H[i][j];
                b = q[j];
                state = 7'b1000110; //Multiplier
                temp_state = 'b000101010;
            end
            else
            begin
                j = 0;
                i = i + 1;
                state = 'b000101001;
            end
        end
        else
        begin
            state = 'b101111010;
        end
    end
    'b000101010:
    begin
        mfhq = Multiplier_Float;
        mfhq = {!mfhq[31], mfhq[30:0]};
        //def_update[i] = Adder_Float(def_update[i],mfhq);
        a = def_update[i];
        b = mfhq;
        state = 'b1000000; //Adder
        temp_state = 'b000101011;
    end
    'b000101011:
    begin
        def_update[i] = Adder_Float;
        j = j + 1;
        state = 'b000101001;
    end
	'b101111010: //Upd
	begin
		//shape_function->update(def_update);
		state = 'b101110101; //Update
		update_temp_state = 'b000101100;
	end
    'b000101100:
    begin
        state_counter = state_counter + 1; 
        guess_u = 32'b0;
        guess_v = 32'b0;
        guess_t = 32'b0;
        //{guess_t, guess_v, guess_u} = map_to_u_v_theta(cx_, cy_);
        //shape_function->map_to_u_v_theta(cx,cy,old_u,old_v,old_t); ??
        //muvt_cx = cx_;
        //muvt_cy = cy_;
        //state = 'b10001101; //map_to_u_v_theta
        //map_to_u_v_theta_temp_state = 'b000101101;
		//aff
		muvt_aff_cx = cx_;
		muvt_aff_cy = cy_;
		state = 'b110010111; //map_to_u_v_theta_aff	
		muvt_aff_temp_state = 'b000101101;
    end
    'b000101101:
    begin
        //guess_u = map_to_u_v_theta[31:0];
        //guess_v = map_to_u_v_theta[63:32];
        //guess_t = map_to_u_v_theta[95:64];
        //SUBSET_DISPLACEMENT_X_FS[gid] = map_to_u_v_theta[31:0];
        //SUBSET_DISPLACEMENT_Y_FS[gid] = map_to_u_v_theta[63:32]; 
        //ROTATION_Z_FS[gid] = map_to_u_v_theta[95:64]; 
		//aff
		guess_u = map_to_u_v_theta_aff[31:0];
		guess_v = map_to_u_v_theta_aff[63:32];
		guess_t = map_to_u_v_theta_aff[95:64];
        //{old_t, old_v, old_u} = map_to_u_v_theta(cx_, cy_);
        //muvt_cx = cx_;
        //muvt_cy = cy_;
        //state = 'b10001101; //map_to_u_v_theta
        //map_to_u_v_theta_temp_state = 'b000101110;
		//aff
		muvt_aff_cx = cx_;
		muvt_aff_cy = cy_;
		state = 'b110010111; //map_to_u_v_theta_aff	
		muvt_aff_temp_state = 'b000101110;
    end
    'b000101110:
    begin
        //old_u = map_to_u_v_theta[31:0];
        //old_v = map_to_u_v_theta[63:32];
        //old_t = map_to_u_v_theta[95:64];
        //SUBSET_DISPLACEMENT_X_FS[gid] = map_to_u_v_theta[31:0];
        //SUBSET_DISPLACEMENT_Y_FS[gid] = map_to_u_v_theta[63:32]; 
        //ROTATION_Z_FS[gid] = map_to_u_v_theta[95:64];
		//aff
		old_u = map_to_u_v_theta_aff[31:0];
		old_v = map_to_u_v_theta_aff[63:32];
		old_t = map_to_u_v_theta_aff[95:64];
		//state = 'b101110110; //test_for_convergence
        //test_for_convergence_temp_state = 'b101111011; //Upd
		//aff
		state = 'b1110001011; //test_for_convergence_aff
		test_for_convergence_aff_temp_state = 'b101111011;
    end
	'b101111011:
	begin
		if(converged)
        begin
            state = 'b000101111;
        end
        else
        begin
            solve_it = solve_it + 1;
            state = 'b1110100011; 
        end
	end
	'b1110100011:
	begin
	   // zero out the storage
	   i = 0;
	   state = 'b1110100100;
	end
	'b1110100100:
	begin
	   if(i <= N)
	   begin
	       q[i] = 32'b0;
	       i = i + 1;
	       state = 'b1110100100;
	   end
	   else
	   begin
	       state = 'b000000111; //solve_it
	   end
	end
    'b000101111:
    begin
        if(solve_it > max_solve_its)
        begin
             MAX_ITERATIONS_REACHED = 1'b1;
             state = 'b000110000;
        end
        else 
        begin //CORRELATION_SUCCESSFUL;
           Correlation_Done = 1'b1;
           state = 'b000110000;
        end 
    end
    'b000110000:
    begin
        if(output_deformed_subset_images_ == 1'b1)
        begin
            //write_deformed_subsets_image(); ****
            gam_done = 1'b0;
        end
        i = 0;
        state = 'b000110001;
    end
    'b000110001:
    begin
        if(i < prev_imgs_size)
        begin
            prev_imgs_[i] = def_imgs_[i];
            i = i + 1;
            state = 'b000110001;
        end
        else
        begin
            state = 'b1110100001;
        end
    end
    'b1110100001:
    begin
        state = 'b1110100000;   //save_fields
        save_fields_temp_state = 'b000110010;
    end
    'b000110010:
    begin
        // accumulate the displacements
        if(use_incremental_formulation_ == 1'b1)
        begin
            i = 0;
            state = 'b000110011;
        end
        else
        begin
            state = 'b101110001;
        end
    end
    'b000110011:
    begin
        if(i < num_of_subsets)
        begin
            //accumulated_disp = Adder_Float(accumulated_disp, SUBSET_DISPLACEMENT_X_FS[i]);
            a = accumulated_disp;
            b = SUBSET_DISPLACEMENT_X_FS[i];
            state = 'b1000000; //Adder
            temp_state = 'b000110100;
        end
        else
        begin
            state = 'b101110001;
        end
    end
    'b000110100:
    begin
        accumulated_disp = Adder_Float;
        //accumulated_disp = Adder_Float(accumulated_disp, SUBSET_DISPLACEMENT_Y_FS[i]);
        a = accumulated_disp;
        b = SUBSET_DISPLACEMENT_Y_FS[i];
        state = 'b1000000; //Adder
        temp_state = 'b000110101;
    end
    'b000110101:
    begin
        accumulated_disp = Adder_Float;
        i = i + 1;
        state = 'b000110011;
    end
    'b000110110:
    begin
        Process_Done = 32'b1;
        //gam_done = 1'b1;
        if(new_frame == 32'b0)
        begin
            gam_idle_counter = gam_idle_counter + 128'b1;
            state = 'b000110110;
        end
        else if(new_frame == 32'b1)
        begin
            Process_Done = 32'b0;
            gam_done = 1'b0;
            gid = 32'b0;
            state = 'b0;
        end
        else if(new_frame == 32'b10)
        begin
            gam_done = 1'b1;
        end
    end
    'b101110001:
    begin
        disp_x[31:0] = SUBSET_DISPLACEMENT_X_FS[gid];
        disp_y[31:0] = SUBSET_DISPLACEMENT_Y_FS[gid]; 
        disp_z[31:0] = ROTATION_Z_FS[gid];
        results_done = 1'b1;
        gid = gid + 1;
        state = 'b101110011;
    end
    'b101110011:
    begin
        state = 'b101110100;
    end
    'b101110100:
    begin
        state = 'b101110010;
    end
    
    /////////////////////////////////////// new functions /////////////////////////////
     7'b1000000: //Adder
     begin
        s1 = a[31];
        s2 = b[31];
        e1 = a[30:23];
        e2 = b[30:23];
        m1[23] = 1'b1;
        m2[23] = 1'b1;
        m1[22:0] = a[22:0];
        m2[22:0] = b[22:0];
        if(e1 == e2)
        begin
            mx = m1;
            my = m2;
            exy = e1+1'b1;
            sign = s1;
        end
        else if(e1 > e2)
        begin
            diff = e1- e2;
            mx = m1;
            my = m2 >> diff;
            exy = e1 + 1'b1;
            sign = s1;
        end
        else
        begin
            diff = e2 - e1;
            mx = m2;
            my = m1 >> diff;
            exy = e2 + 1'b1;
            sign = s2;
        end
        sr = s1 ^ s2;
        state = 7'b1000001;
     end
     7'b1000001: //3'b001:
     begin
         if(sr == 0)
         begin
             mxy = mx + my;
             sign = s1;
         end
         else
         begin
             if(mx >= my)
                mxy = mx - my;
             else
                mxy = my - mx;
         end
         mxy2 = mxy;
         if(s1 == 0 && s2 == 0)
            sign=1'b0;
         else if (s1 == 1 && s2 == 1)
            sign=1'b1;
         else if (s1 == 0 && s2 == 1)
         begin
             if(e1 < e2 || ((e1 == e2) && (m1 < m2)))
                sign = 1'b1;
             else
                sign = 1'b0;
         end 
         else
         begin
             if(e1 < e2 || ((e1 == e2) && (m1 < m2)))
                sign = 1'b0;
             else
                sign = 1'b1;
         end
         state = 7'b1000010;
     end
     7'b1000010: //3'b010:
     begin
         for(i_for = 0; i_for < 12; i_for = i_for + 1)
            if (mxy[24] == 0)
            begin
                mxy = mxy << 1;
                exy = exy - 1;
            end
         state = 7'b1000011;
     end
     7'b1000011: //3'b011:
     begin
        for(i_for = 12; i_for < 24; i_for = i_for + 1)
            if (mxy[24] == 0)
            begin
                mxy = mxy << 1;
                exy = exy - 1;
            end
        state = 7'b1000100;                        
     end
     7'b1000100: //3'b100:
     begin
        if(a[30:0] == 31'b0000000000000000000000000000000)
            Adder_Float = b[31:0];
        else if (b[30:0] == 31'b0000000000000000000000000000000)
            Adder_Float = a[31:0];
        else
            Adder_Float = {sign, exy, mxy[23:1]};
        done = 1'b1;
        Subtractor_Float = Adder_Float;
        state = temp_state;
     end
     7'b1000101: //Subtractor
     begin
         if (b[30:0] == 31'b0)
         begin
             Subtractor_Float = a[31:0];
             state = temp_state;
         end
         else if(a[31:0] == b[31:0])
         begin
             Subtractor_Float = 32'b0;
             state = temp_state;
         end
         else
         begin
             b[31:0] ={{ !b[31]}, {b[30:0]}};
             //Subtractor_Float = Adder_Float(a,b);
             state = 7'b1000000;
         end
     end
    9'b1000110: // Multiplier //b1000110
    begin
        s1 = a[31]; //Sign bit
        s2 = b[31]; 
        e1 = a[30:23]; // Exponent bits
        e2 = b[30:23];
        m1[23] = 1'b1; //Mantissa 24th bit should be 1
        m2[23] = 1'b1;
        m1[22:0] = a[22:0]; // Mantissa bits
        m2[22:0] = b[22:0];
        state = 9'b1000111;
    end
    9'b1000111:
    begin
        e_sum = e1 + e2;  //Exponent addition +1'b1
        exponent= e_sum - 8'b01111110; //01111111
        state = 9'b1001000;
    end
    9'b1001000:
    begin
        if ( a!=0 || b!=0 )
        begin
            prod = m1*m2;    // mantissa product
            product = prod[47:24];
        end
        state = 9'b1001001;
    end
    9'b1001001:
    begin
        if(product == 0)
        begin
            Multiplier_Float=32'b0;
            state = 9'b1001011;
        end
        else
        begin
            //Normalization
            for(i_for = 0;i_for < 12;i_for = i_for + 1)
                if (product[23]== 0)
                begin
                    product = product << 1;
                    exponent = exponent - 1;
                end
            state = 9'b101101111;
        end //else
    end
    'b101101111:
    begin
        //Normalization
        for(i_for = 12;i_for < 23;i_for = i_for + 1)
            if (product[23]== 0)
            begin
                product = product << 1;
                exponent = exponent - 1;
            end
        state = 9'b1001010;
    end
    9'b1001010: //4'b0100:
    begin
        sign= s1 ^ s2; // Sign Calculation
        if(a[30:0] == 31'b0 || b[30:0] == 31'b0) // if any input is 0, output is 0
        begin
            Multiplier_Float = 32'b0;
        end
        else
        begin
            Multiplier_Float = {sign,exponent,product[22:0]}; // Output
        end
        state = 9'b1001011;         
    end
    9'b1001011: //101
    begin
        multiplier_done = 1'b1;
        state = temp_state;
    end 
    7'b1001100: //Divider
    begin
        x_mantissa = a[22:0];
        x_exponent = a[30:23];
        x_sign = a[31];
        y_mantissa = b[22:0];
        y_exponent = b[30:23];
        y_sign = b[31];
        z_sign = x_sign ^ y_sign;
        state = 7'b1001101;
    end
    7'b1001101:
    begin
        if (y_exponent==8'b11111111) 
        begin //x/inf = 0
            z_exponent = 8'b00000000;
            z_mantissa = 23'b00000000000000000000000;
        end
        else if (y_exponent==0 || x_exponent==255) 
        begin // result = infinity
            // x/0 or inf/x = inf
            z_exponent = 8'b11111111;
            z_mantissa = 23'b00000000000000000000000;
        end
        else
        begin
            exponent_aux = {{1'b0}, x_exponent} - {{1'b0}, y_exponent} + 127;
        end
        state = 7'b1001110;
    end
    7'b1001110: //4'b0010:
    begin
        partial_remainder = {{2'b01}, x_mantissa}; // P = 1.F1
        i_for = 25;
        state = 7'b1001111;
    end
    7'b1001111: //4'b0011:
    begin  
        if( i_for >= 0)
        begin
            tmp_remainder = partial_remainder - {{2'b01}, y_mantissa};
            if ( tmp_remainder[24]==1'b0 )
            begin 
                aa[i_for]=1'b1;
                partial_remainder = tmp_remainder;
            end
            else
            begin
                aa[i_for]=1'b0;
            end 
            partial_remainder = {{partial_remainder[23:0]}, {1'b0}};
            i_for = i_for - 1;
            state = 'b1001111;
        end
        else
        begin
            state = 'b1010000;
        end
    end
    7'b1010000: //4'b0100:
    begin
        aa = aa + 1; // round
        if (aa[25]==1'b1)
        begin 
            z_mantissa = aa[24:2];
        end
        else
        begin
            z_mantissa = aa[23:1];
            exponent_aux = exponent_aux - 1;
        end
        state = 7'b1010001;
    end
    9'b1010001: //4'b0101:
    begin
        if (exponent_aux[8] == 1'b1)
        begin
            if (exponent_aux[7] == 1'b1) // underflow
            begin
                z_exponent = 9'b00000000;
                z_mantissa = 23'b00000000000000000000000;
            end
            else // overflow
            begin
                z_exponent = 9'b11111111;
                z_mantissa = 23'b00000000000000000000000;
            end
        end    
        else
        begin
            z_exponent = exponent_aux[7:0];
        end
        state = 7'b1010010;    
    end
    9'b1010010: //4'b0110:
    begin
        Divider_Float[31:0] = {{z_sign}, {z_exponent}, {z_mantissa}}; 
        state = temp_state;   
    end   
    7'b1010011: //4'b0000: SQRT
    begin
        x_mantissa = a[22:0];
        x_exponent = a[30:23];
        x_sign = a[31];
        y_sign = 1'b0;
        state = 7'b1010100;
    end
    7'b1010100: //4'b0001:
    begin
        if (x_exponent == 8'b00000000) 
        begin // zero
            y_exponent = 8'b00000000;
            y_mantissa = 23'b000000000000000000000000;
            state = 7'b1011001; //4'b0110;
        end
        else if (x_exponent == 8'b11111111) 
        begin // infinity
            y_exponent = 8'b11111111;
            y_mantissa = 23'b000000000000000000000000;
            state = 7'b1011001; //4'b0110;
        end
        else 
        begin
            state = 7'b1010101; //4'b0010;
        end   
    end
    7'b1010101: //4'b0010:
    begin
        if (x_exponent[0]==1'b1) 
        begin // exponent-127 is even
            y_exponent = {{1'b0},  x_exponent[7:1]} + 64;
            ix = {{2'b01} , x_mantissa [22:0], {1'b0}} ;
        end
        else 
        begin // exponent-127 is odd
            // shit mantissa one to the left and subtract one from x_exponent
            y_exponent = {{1'b0}, {x_exponent [7:1]}} + 63;
            ix = {{1'b1}, x_mantissa [22:0], {2'b00}};
        end //else
        state = 7'b1010110; //4'b0011;    
    end
    7'b1010110: //4'b0011:
    begin
        aa_sqrt = 52'b0000000000000000000000000000000000000000000000000000;
        biti = {{2'b10}, {24'b000000000000000000000000}}; //"10" & x"000000"; -- 2^(25)//?????????????
        r[51:26]= ix; // r is in Q26
        r[25:0]= 26'b00000000000000000000000000;
        i_for = 25;
        state = 7'b1010111; //4'b0100;
    end
    7'b1010111: //4'b0100:
        begin
            if(i_for >= 0)
            begin
                rt = ({{1'b0} , {r[51:0]}}) - ({{1'b0}, {aa_sqrt ^ bit2bit_sq(biti)}});
                aa_sqrt = {{1'b0}, {aa_sqrt[51:1]}};
                state = 'b101101101;
            end
            else
            begin
                state = 'b1011000; //4'b0101; 
            end   
        end
        'b101101101:
        begin
            if (rt[52]== 1'b0) 
            begin // rt>=0
                r = rt[51:0];
                aa_sqrt = (aa_sqrt ^ bit2bit_sq(biti)); //a or bit2bit_sq(biti); -- the adder is safelly replaced by an or
            end
            biti = {{1'b0} ,biti[25:1]}; //'0' & biti(25 downto 1); -- srl 1
            i_for = i_for - 1;
            state = 'b1010111;
        end
    7'b1011000: //4'b0101:
    begin
        aa_sqrt[24:2] = aa_sqrt[24:2]+aa_sqrt[1]; // round
        y_mantissa = aa_sqrt[24:2];
        state = 7'b1011001; //4'b0110;
    end
    7'b1011001: //4'b0110:
    begin
        SQRT[22:0] = y_mantissa;
        SQRT[30:23] = y_exponent;
        SQRT[31] = y_sign;
        state = temp_state;
    end
    9'b1011010: //cos
    begin
        a = a; 
        b = a;
        //t1 = Multiplier_Float (x, x); //x^2
        state = 9'b1000110; //Multiplier
        temp_state = 9'b1011011; 
    end
    9'b1011011:
    begin
        t1 = Multiplier_Float;
        //t2 = Multiplier_Float (t1, t1); //x^4 
        a = t1;
        b = t1;
        state = 9'b1000110; //Multiplier
        temp_state = 9'b1011100;
    end
    9'b1011100:
    begin
        t2 = Multiplier_Float;
        //t1 = Divider_Float(t1, 32'b01000000000000000000000000000000); //x^2 / 2
        a = t1;
        b = 32'b01000000000000000000000000000000;
        state = 9'b1001100; //Divider
        temp_state = 9'b1011101;
    end
    9'b1011101:
    begin
        t1 = Divider_Float;
        //t2 = Divider_Float(t2, 32'b01000001110000000000000000000000); //x^4 / 24
        a = t2;
        b = 32'b01000001110000000000000000000000;
        state = 9'b1001100; //Divider
        temp_state = 9'b1011110;
    end
    9'b1011110:
    begin
        t2 = Divider_Float;
        //t1 = Subtractor_Float(32'b00111111100000000000000000000000, t1); //1 - x^2/2
        a = 32'b00111111100000000000000000000000;
        b = t1;
        state = 9'b1000101; //Subtractor
        temp_state = 9'b1011111;
    end
    9'b1011111:
    begin
        t1 = Subtractor_Float;
        //cos = Adder_Float(t1, t2);
        a = t1;
        b = t2;
        state = 9'b1000000; //Adder
        temp_state = 9'b1100000;
    end
    9'b1100000:
    begin
        cos = Adder_Float;
        state = cos_temp_state;    
        cos_done = 1'b1;
    end
    7'b1100001: //sin
    begin
        //t1 = Multiplier_Float (x, x); //x^2
        a = a;
        b = a;
        a_tmp = a;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1100010;
    end
    7'b1100010:
    begin
        t1 = Multiplier_Float;
        //t2 = Multiplier_Float (t1, x); //x^3 t2
        a = t1;
        b = a_tmp;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1100011;
    end
    7'b1100011:
    begin
        t2 = Multiplier_Float;
        //t3 = Multiplier_Float (t1, t2); //x^5
        a = t1;
        b = t2;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1100100;
    end
    7'b1100100:
    begin
        t3 = Multiplier_Float;
        //t2 = Divider_Float(t2, 32'b01000000110000000000000000000000); //x^3 / 6
        a = t2;
        b = 32'b01000000110000000000000000000000;
        state = 7'b1001100; //Divider
        temp_state = 7'b1100101;
    end
    7'b1100101:
    begin
        t2 = Divider_Float;
        //t3 = Divider_Float(t3, 32'b01000010111100000000000000000000); //x^5 / 120
        a = t3;
        b = 32'b01000010111100000000000000000000;
        state = 7'b1001100; //Divider
        temp_state = 7'b1100110;  
    end
    7'b1100110:
    begin
        t3 = Divider_Float;
        //t1 = Subtractor_Float(x, t2); //x - x^3/6
        a = a_tmp;
        b = t2;
        state = 7'b1000101; //Subtractor
        temp_state = 7'b1100111;
    end
    7'b1100111:
    begin
        t1 = Subtractor_Float;
        //sin = Adder_Float(t1, t3);
        a = t1;
        b = t3;
        state = 7'b1000000; //Adder
        temp_state = 7'b1101000;
    end
    7'b1101000:
    begin
        sin = Adder_Float;
        state = sin_temp_state;
        sin_done = 1'b1;
    end
    7'b1101001: //asin
    begin
        //t1 = Multiplier_Float(x, x);
        a = a;
        b = a;
        a_tmp = a;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1101010; 
    end
    7'b1101010:
    begin
        t1 = Multiplier_Float;
        //t2 = Multiplier_Float(t1, x); // x^3
        a = t1;
        b = a_tmp;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1101011; 
    end
    7'b1101011:
    begin
       t2 = Multiplier_Float;
       //t1 = Multiplier_Float(t1, t2); //x^5
       a = t1;
       b = t2;
       state = 7'b1000110; //Multiplier
       temp_state = 7'b1101100;  
    end
    7'b1101100:
    begin
        t1 = Multiplier_Float;
        //t2 = Divider_Float(t2,32'b01000000110000000000000000000000); //x^3 / 6
        a = t2;
        b = 32'b01000000110000000000000000000000;
        state = 7'b1001100; //Divider
        temp_state = 7'b1101101;  
    end
    7'b1101101:
    begin
        t2 = Divider_Float;
        //t1 = Multiplier_Float(t1,32'b01000000010000000000000000000000); //x^5 * 3
        a = t1;
        b = 32'b01000000010000000000000000000000;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1101110; 
    end
    7'b1101110:
    begin
        t1 = Multiplier_Float;
        //t1 = Divider_Float(t1,32'b01000010001000000000000000000000); //3 * x^5 / 40
        a = t1;
        b = 32'b01000010001000000000000000000000;
        state = 7'b1001100; //Divider
        temp_state = 7'b1101111;
    end
    7'b1101111:
    begin
        t1 = Divider_Float;
        //t1 = Adder_Float(t1, t2);
        a = t1;
        b = t2;
        state = 7'b1000000; //Adder
        temp_state = 7'b1110000;
    end
    7'b1110000:
    begin
        t1 = Adder_Float;
        //asin = Adder_Float(x, t1);
        a = a_tmp;
        b = t1;
        state = 7'b1000000; //Adder
        temp_state = 7'b1110001;
    end
    7'b1110001:
    begin
        asin = Adder_Float;
        state = asin_temp_state;
        asin_done = 1'b1;
    end
    7'b1110010: //acos
    begin
        //t1 = Multiplier_Float(x, x);
        a = a; 
        b = a;
        a_tmp = a;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1110011;  
    end
    7'b1110011:
    begin
        t1 = Multiplier_Float;
        //t2 = Multiplier_Float(t1, x); // x^3
        a = a_tmp;
        b = t1;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1110100; 
    end
    7'b1110100:
    begin
        t2 = Multiplier_Float;
        //t1 = Multiplier_Float(t1, t2); //x^5
        a = t1;
        b = t2;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1110101; 
    end
    7'b1110101:
    begin
        t1 = Multiplier_Float;
        //t2 = Divider_Float(t2,32'b01000000110000000000000000000000); //x^3 / 6
        a = t2; 
        b = 32'b01000000110000000000000000000000;
        state = 7'b1001100; //Divider
        temp_state = 7'b1110110; 
    end
    7'b1110110:
    begin
        t2 = Divider_Float;
        //t1 = Multiplier_Float(t1,32'b01000000010000000000000000000000); //x^5 * 3
        a = t1; 
        b = 32'b01000000010000000000000000000000;
        state = 7'b1000110; //Multiplier
        temp_state = 7'b1110111; 
    end
    7'b1110111:
    begin
        t1 = Multiplier_Float;
        //t1 = Divider_Float(t1,32'b01000010001000000000000000000000); //3 * x^5 / 40
        a = t1;
        b = 32'b01000010001000000000000000000000;
        state = 7'b1001100; //Divider
        temp_state = 7'b1111000;
    end
    7'b1111000:
    begin
        t1 = Divider_Float;
        //t3 = Subtractor_Float(32'b00111111110010001111010111000011,x); //pi/2 - x
        a = 32'b00111111110010001111010111000011;
        b = a_tmp;
        state = 7'b1000101; //Subtractor
        temp_state = 7'b1111001;
    end
    7'b1111001:
    begin
        t3 = Subtractor_Float;
        //t2 = Subtractor_Float(t3, t2); //pi/2 - x - x^3 / 6 
        a = t3;
        b = t2;
        state = 7'b1000101; //Subtractor
        temp_state = 7'b1111010;
    end
    7'b1111010:
    begin
        t2 = Subtractor_Float;
        //acos = Subtractor_Float(t2, t1); //pi/2 - x - x^3 / 6  -3 * x^5 / 40 
        a = t2;
        b = t1;
        state = 7'b1000101; //Subtractor
        temp_state = 7'b1111011;
    end
    7'b1111011:
    begin
        acos = Subtractor_Float;
        state = acos_temp_state;
        acos_done = 1'b1;
    end
    7'b1111100: //gamma_
    begin
        gamma_ = 32'b0;
        gamma_i = 0;
        state = 'b1111101;
    end
    'b1111101:
    begin
        if(gamma_i < num_pxl_Int) //num_pixels_
        begin
            // The current version does not support this feature (pixel deactivation)
            /*if(is_active_[gamma_i] == 1'b1)
            begin
                //{mean_def, mean_sum_def} = mean(1'b1);
                mean_ref_def = 1'b1;
                state = 'b11000010; //mean
                mean_temp_state = 'b1111110; 
            end
            else
            begin
                gamma_i = gamma_i + 1;
                state = 'b1111101;
            end*/
            
            //{mean_def, mean_sum_def} = mean(1'b1);
            mean_ref_def = 1'b1;
            state = 'b11000010; //mean
            mean_temp_state = 'b1111110; 
        end
        else
        begin
            state = 'b10000111;   
        end
    end
    'b1111110:
    begin
        gamma_mean_def = mean[63:32];
        gamma_mean_sum_def = mean[31:0];
        //{mean_ref, mean_sum_ref} = mean(1'b0); //ref
        mean_ref_def = 1'b0;
        state = 'b11000010; //mean
        mean_temp_state = 'b1111111; 
    end
    'b1111111:
    begin
        gamma_mean_sum_ref = mean[31:0];
        gamma_mean_ref = mean[63:32];
        //t1 = Subtractor_Float(def_intensities_[i], mean_def);
        if(clk_counter_b == 2'b10)
        begin
            a = dout_def_ints;
            clk_counter_b =  2'b00;
            b = gamma_mean_def;
            state = 7'b1000101; //Subtractor
            temp_state = 'b10000000;                   
        end
        else
        begin
            addr_def_ints = gamma_i;
            clk_counter_b = clk_counter_b + 1;
            state = 'b1111111;
        end
    end
    'b10000000:
    begin
        gamma_t1 = Subtractor_Float;
        //t1 = Divider_Float(t1, mean_sum_def);
        a = gamma_t1;
        b = gamma_mean_sum_def;
        state = 7'b1001100; //Divider
        temp_state = 'b10000001; 
    end
    'b10000001:
    begin
        gamma_t1 = Divider_Float;
        //t2 = Subtractor_Float(ref_intensities_[i], mean_ref);
        if(clk_counter_a == 2'b10)
        begin
            a = dout_ref_ints;
            clk_counter_a =  2'b00;
            b = gamma_mean_ref;
            state = 7'b1000101; //Subtractor
            temp_state = 'b10000010;                     
        end
        else
        begin
            addr_ref_ints = gamma_i;
            clk_counter_a = clk_counter_a + 1;
            state = 'b10000001;
        end
    end
    'b10000010:
    begin
        gamma_t2 = Subtractor_Float;
        //t2 = Divider_Float(t2, mean_sum_ref);
        a = gamma_t2;
        b = gamma_mean_sum_ref;
        state = 7'b1001100; //Divider
        temp_state = 'b10000011; 
    end
    'b10000011:
    begin
        gamma_t2 = Divider_Float;
        //value = Subtractor_Float(t1, t2); 
        a = gamma_t1;
        b = gamma_t2;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10000100; 
    end
    'b10000100:
    begin
        value = Subtractor_Float;
        //temp_gamma = Multiplier_Float(value, value);
        a = value;
        b = value;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10000101; 
    end
    'b10000101:
    begin
        temp_gamma = Multiplier_Float;
        //gamma_ = Adder_Float(gamma_, temp_gamma);
        a = gamma_;
        b = temp_gamma;
        state = 7'b1000000; //Adder
        temp_state = 'b10000110; 
    end
    'b10000110:
    begin
        gamma_ = Adder_Float;
        gamma_i = gamma_i + 1;
        state = 'b1111101; 
    end 
    'b10000111:
    begin
        if(gamma_mean_sum_ref == 32'b0 || gamma_mean_sum_def == 32'b0)
        begin
            gamma_ = 32'b10111111100000000000000000000000; //-1.0;
        end
        state = gamma_temp_state;
    end
    'b10001000: //insert_motion
    begin
        QUAD_F_FS = im_u;
        QUAD_L_FS = im_v;
        //QUAD_A_FS = cos(theta);
        a = im_theta;
        state = 'b1011010; //cos
        cos_temp_state = 'b10001001; 
    end
    'b10001001:
    begin
        QUAD_A_FS = cos;
        //QUAD_B_FS = sin(theta);
        a = im_theta;
        state = 'b1100001; //sin
        sin_temp_state = 'b10001010; 
    end 
    'b10001010:
    begin
        QUAD_B_FS = sin;
        QUAD_B_FS[31] = 1'b1;
        //QUAD_G_FS = sin(theta);
        a = im_theta;
        state = 'b1100001; //sin
        sin_temp_state = 'b10001011;
    end
    'b10001011:
    begin
        QUAD_G_FS = sin;
        //QUAD_H_FS = cos(theta);
        a = im_theta;
        state = 'b1011010; //cos
        cos_temp_state = 'b10001100;
    end
    'b10001100:
    begin
        QUAD_H_FS = cos;
        QUAD_C_FS = 32'b0;
        QUAD_D_FS = 32'b0;
        QUAD_E_FS = 32'b0;
        QUAD_I_FS = 32'b0; 
        QUAD_J_FS = 32'b0; 
        QUAD_K_FS = 32'b0;
        insert_motion = 1'b1;
        state = insert_motion_temp_state;
    end
    'b10001101: //map_to_u_v_theta muvt_
    begin
        muvt_cxp = 32'b0;
        muvt_cxy = 32'b0;
        //{cyp, cxp} = map(cx,cy,cx,cy);
        map_x = muvt_cx;
        map_y = muvt_cy;
        map_cx = muvt_cx;
        map_cy = muvt_cy;
        state = 'b10100011; //map
        map_temp_state = 'b10001110;
    end
    'b10001110:
    begin
        muvt_cxp = map[31:0];
        muvt_cyp = map[63:32];
        //map_to_u_v_theta[31:0] = Subtractor_Float(cxp, cx); //out_u
        a = muvt_cxp;
        b = muvt_cx;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10001111;
    end
    'b10001111:
    begin
        map_to_u_v_theta[31:0] = Subtractor_Float;
        //map_to_u_v_theta[63:32] = Subtractor_Float(cyp, cy); //out_v
        a = muvt_cyp;
        b = muvt_cy;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10010000;
    end
    'b10010000:
    begin
        map_to_u_v_theta[63:32] = Subtractor_Float;
        muvt_rxp = 32'b0;
        muvt_ryp = 32'b0;
        //cx_5 = Adder_Float(cx, 32'b01000000101000000000000000000000); //cx+5
        a = muvt_cx;
        b = 32'b01000000101000000000000000000000;
        state = 7'b1000000; //Adder
        temp_state = 'b10010001;
    end
    'b10010001:
    begin
        muvt_cx_5 = Adder_Float;
        //{rxp, ryp} = map(cx_5,cy,cx,cy);
        map_x = muvt_cx_5;
        map_y = muvt_cy;
        map_cx = muvt_cx;
        map_cy = muvt_cy;
        state = 'b10100011; //map
        map_temp_state = 'b10010010;
    end
    'b10010010:
    begin
        muvt_ryp = map[31:0];
        muvt_rxp = map[63:32];
        //ax = Subtractor_Float(rxp, cxp);
        a = muvt_rxp;
        b = muvt_cxp;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10010011;
    end
    'b10010011:
    begin
        muvt_ax = Subtractor_Float;
        //ay = Subtractor_Float(ryp, cyp);
        a = muvt_ryp;
        b = muvt_cyp;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10010100;
    end
    'b10010100:
    begin
        muvt_ay = Subtractor_Float;
        //ax_s = Multiplier_Float(ax, ax);
        a = muvt_ax;
        b = muvt_ax;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10010101;
    end
    'b10010101:
    begin
        muvt_ax_s = Multiplier_Float;
        //ay_s = Multiplier_Float(ay, ay);
        a = muvt_ay;
        b = muvt_ay;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10010110;
    end
    'b10010110:
    begin
        muvt_ay_s = Multiplier_Float;
        //mag_a = Adder_Float(ax_s, ay_s);
        a = muvt_ax_s;
        b = muvt_ay_s;
        state = 7'b1000000; //Adder
        temp_state = 'b10010111;
    end
    'b10010111:
    begin
        muvt_mag_a = Adder_Float;
        //mag_a = SQRT(mag_a);
        a = muvt_mag_a;
        state = 7'b1010011; //SQRT
        temp_state = 'b10011000;
    end
    'b10011000:
    begin
        muvt_mag_a = SQRT;
        muvt_bx = 32'b01000000101000000000000000000000; //5
        muvt_by = 32'b0;
        muvt_mag_b = 32'b01000000101000000000000000000000; //5
        //bx = Multiplier_Float(ax, bx); //to do not declare extra variables
        a = muvt_ax;
        b = muvt_bx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10011001;     
    end
    'b10011001:
    begin
        muvt_bx = Multiplier_Float;
        //by = Multiplier_Float(ay, by);
        a = muvt_ay;
        b = muvt_by;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10011010; 
    end
    'b10011010:
    begin
        muvt_by = Multiplier_Float;
        //bx = Adder_Float(bx, by);
        a = muvt_bx;
        b = muvt_by;
        state = 7'b1000000; //Adder
        temp_state = 'b10011011; 
    end
    'b10011011:
    begin
        muvt_bx = Adder_Float;
        //mag_b = Multiplier_Float(mag_a, mag_b);
        a = muvt_mag_a;
        b = muvt_mag_b;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10011100;
    end
    'b10011100:
    begin
        muvt_mag_b = Multiplier_Float;
        //a_dot_b_over_mags = Divider_Float(bx, mag_b);
        a = muvt_bx;
        b = muvt_mag_b;
        state = 7'b1001100; //Divider
        temp_state = 'b10011101;
    end
    'b10011101:
    begin
        muvt_a_dot_b_over_mags = Divider_Float;
        if(muvt_ay[31] == 1'b1)
        begin
            state = 'b10011110; 
        end
        else
        begin
            state = 'b10100001; 
        end
    end
    'b10011110:
    begin
        //a_dot_b_over_mags = acos(a_dot_b_over_mags);
        a = muvt_a_dot_b_over_mags;
        state = 'b1110010; //acos
        acos_temp_state = 'b10011111;
    end
    'b10011111:
    begin
        muvt_a_dot_b_over_mags = acos;
        //map_to_u_v_theta[95:64] = Subtractor_Float(32'b01000000110010010000111111011010, a_dot_b_over_mags);//2*pi
        a = 32'b01000000110010010000111111011010;
        b = muvt_a_dot_b_over_mags;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10100000;
    end
    'b10100000:
    begin
        map_to_u_v_theta[95:64] = Subtractor_Float;
        state = 'b10100010;
    end
     'b10100001:
     begin
        //map_to_u_v_theta[95:64] = acos(a_dot_b_over_mags);
        a = muvt_a_dot_b_over_mags;
        state = 'b1110010; //acos
        acos_temp_state = 'b101101110;   
     end
     'b101101110:
     begin
        map_to_u_v_theta[95:64] = acos;
        state = 'b10100010;
     end
     'b10100010:
     begin
        state = map_to_u_v_theta_temp_state;
     end
     'b10100011: //map
     begin
        //dx = Subtractor_Float(x, cx); //dx
        a = map_x;
        b = map_cx;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10100100; 
     end
     'b10100100:
     begin
        map_dx = Subtractor_Float;
        //dy = Subtractor_Float(y, cy); //dy
        a = map_y;
        b = map_cy;
        state = 7'b1000101; //Subtractor
        temp_state = 'b10100101;
     end
     'b10100101:
     begin
        map_dy = Subtractor_Float;
        //t1 = Multiplier_Float(QUAD_A_FS, dx);
        a = QUAD_A_FS;
        b = map_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10100110;
     end
     'b10100110:
     begin
        map_t1 = Multiplier_Float;
        //t2 = Multiplier_Float(QUAD_B_FS, dy);
        a = QUAD_B_FS;
        b = map_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10100111;
     end
     'b10100111:
     begin
        map_t2 = Multiplier_Float;
        //t1 = Adder_Float(t1, t2); // parameter(QUAD_A_FS)*dx + parameter(QUAD_B_FS)*dy
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10101000;
     end 
     'b10101000:
     begin
        map_t1 = Adder_Float;
        //t2 = Multiplier_Float(dx, dy);
        a = map_dx;
        b = map_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10101001;
     end
     'b10101001:
     begin
        map_t2 = Multiplier_Float;
        //t2 = Multiplier_Float(t2, QUAD_C_FS);
        a = map_t2;
        b = QUAD_C_FS;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10101010;
     end
     'b10101010:
     begin
        map_t2 = Multiplier_Float;
        //t1 = Adder_Float(t1, t2); //parameter(QUAD_A_FS)*dx + parameter(QUAD_B_FS)*dy + parameter(QUAD_C_FS)*dx*dy
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10101011;
     end
     'b10101011:
     begin
        map_t1 = Adder_Float;
        //t2 = Multiplier_Float(dx, dx);
        a = map_dx;
        b = map_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10101100;
     end
     'b10101100:
     begin
        map_t2 = Multiplier_Float;
        //t2 = Multiplier_Float(t2, QUAD_D_FS);
        a = map_t2;
        b = QUAD_D_FS;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10101101;
     end
     'b10101101:
     begin
        map_t2 = Multiplier_Float;
        //t2 = Adder_Float(t1, t2); //parameter(QUAD_A_FS)*dx + parameter(QUAD_B_FS)*dy + parameter(QUAD_C_FS)*dx*dy + parameter(QUAD_D_FS)*dx*dx 
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10101110;
     end
     'b10101110:
     begin
        map_t2 = Adder_Float;
        //t1 = Multiplier_Float(dy, dy);
        a = map_dy;
        b = map_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10101111; 
     end
     'b10101111:
     begin
        map_t1 = Multiplier_Float;
        //t1 = Multiplier_Float(t1, QUAD_E_FS);
        a = map_t1;
        b = QUAD_E_FS;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10110000; 
     end
     'b10110000:
     begin
        map_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, t2); 
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10110001; 
     end
     'b10110001:
     begin
        map_t1 = Adder_Float;
        //t1 = Adder_Float(t1, QUAD_F_FS);
        a = map_t1;
        b = QUAD_F_FS;
        state = 7'b1000000; //Adder
        temp_state = 'b10110010; 
     end
     'b10110010:
     begin
        map_t1 = Adder_Float;
        //map[31:0] = Adder_Float(t1, cx);
        a = map_t1;
        b = map_cx;
        state = 7'b1000000; //Adder
        temp_state = 'b10110011; 
     end
     'b10110011:
     begin
        map[31:0] = Adder_Float;
        //t1 = Multiplier_Float(QUAD_G_FS, dx);
        a = QUAD_G_FS;
        b = map_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10110100;
     end
     'b10110100:
     begin
        map_t1 = Multiplier_Float;
        //t2 = Multiplier_Float(QUAD_H_FS, dy);
        a = QUAD_H_FS;
        b = map_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10110101;
     end
     'b10110101:
     begin
        map_t2 = Multiplier_Float;
        //t1 = Adder_Float(t1, t2);
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10110110;
     end
     'b10110110:
     begin
        map_t1 = Adder_Float;
        //t2 = Multiplier_Float(dx, dy);
        a = map_dx;
        b = map_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10110111;
     end
     'b10110111:
     begin
        map_t2 = Multiplier_Float;
        //t2 = Multiplier_Float(t2, QUAD_I_FS);
        a = map_t2;
        b = QUAD_I_FS;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10111000;
     end
     'b10111000:
     begin
        map_t2 = Multiplier_Float;
        //t1 = Adder_Float(t1, t2);
        a = map_t1;
        b = map_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10111001;
     end
     'b10111001:
     begin
        map_t1 = Adder_Float;
        //t2 = Multiplier_Float(dx, dx);
        a = map_dx;
        b = map_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10111010;
     end
     'b10111010:
     begin
        map_t2 = Multiplier_Float;
        //t2 = Multiplier_Float(t2, QUAD_J_FS);
        a = map_t2;
        b = QUAD_J_FS;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10111011;
     end
     'b10111011:
     begin
        map_t2 = Multiplier_Float;
        //t2 = Adder_Float(t1, t2);
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10111100;
     end
     'b10111100:
     begin
        map_t2 = Adder_Float;
        //t1 = Multiplier_Float(dy, dy);
        a = map_dy;
        b = map_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10111101;
     end
     'b10111101:
     begin
        map_t1 = Multiplier_Float;
        //t1 = Multiplier_Float(t1, QUAD_K_FS);
        a = map_t1;
        b = QUAD_K_FS;
        state = 7'b1000110; //Multiplier
        temp_state = 'b10111110;
     end
     'b10111110:
     begin
        map_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, t2);
        a = map_t1;
        b = map_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b10111111;
     end
     'b10111111:
     begin
        map_t1 = Adder_Float;
        //t1 = Adder_Float(t1, QUAD_L_FS); 
        a = t1;
        b = QUAD_L_FS;
        state = 7'b1000000; //Adder
        temp_state = 'b11000000;
     end
     'b11000000:
     begin
        map_t1 = Adder_Float;
        //map[63:32] = Adder_Float(t1, cy); //map[63:31]
        a = map_t1;
        b = map_cy;
        state = 7'b1000000; //Adder
        temp_state = 'b11000001;
     end
     'b11000001:
     begin 
         map[63:32] = Adder_Float;
         state = map_temp_state;
     end
     'b11000010: //mean
     begin
        mean_temp_sum = 32'b0;
        i_mean = 0;
        state = 'b11000011;
     end
     'b11000011:
     begin
        if(i_mean < num_pxl_Int) //for each subset
        begin
                   
            // The current version does not support this feature (pixel deactivation)
            /*if(subset_is_active [i_mean] == 1'b1)
            begin
                if(mean_ref_def == 1'b0)
                begin
                    //temp_sum = Adder_Float(temp_sum, ref_intensities_[i*32 + 31-:32]);                   
                    if(clk_counter_b == 2'b10)
                    begin
                        b = dout_ref_ints;
                        clk_counter_b =  2'b00;
                        a = mean_temp_sum;
                        state = 7'b1000000; //Adder
                        temp_state = 'b11000100;                    
                    end
                    else
                    begin
                        addr_ref_ints = i_mean;
                        clk_counter_b = clk_counter_b + 1;
                        state = 'b11000011;
                    end
                end
                else
                begin
                    //temp_sum = Adder_Float(temp_sum, def_intensities_[i*32 + 31-:32]);             
                    if(clk_counter_b == 2'b10)
                    begin
                        b = dout_def_ints;
                        clk_counter_b =  2'b00;
                        a = mean_temp_sum;
                        state = 7'b1000000; //Adder
                        temp_state = 'b11000100;                    
                    end
                    else
                    begin
                        addr_def_ints = i_mean;
                        clk_counter_b = clk_counter_b + 1;
                        state = 'b11000011;
                    end
                end
            end
            else
            begin
                i_mean = i_mean + 1;
                state = 'b11000011;
            end*/

            if(mean_ref_def == 1'b0)
            begin
                //temp_sum = Adder_Float(temp_sum, ref_intensities_[i*32 + 31-:32]);                   
                if(clk_counter_b == 2'b10)
                begin
                    b = dout_ref_ints;
                    clk_counter_b =  2'b00;
                    a = mean_temp_sum;
                    state = 7'b1000000; //Adder
                    temp_state = 'b11000100;                    
                end
                else
                begin
                    addr_ref_ints = i_mean;
                    clk_counter_b = clk_counter_b + 1;
                    state = 'b11000011;
                end
            end
            else
            begin
                //temp_sum = Adder_Float(temp_sum, def_intensities_[i*32 + 31-:32]);             
                if(clk_counter_b == 2'b10)
                begin
                    b = dout_def_ints;
                    clk_counter_b =  2'b00;
                    a = mean_temp_sum;
                    state = 7'b1000000; //Adder
                    temp_state = 'b11000100;                    
                end
                else
                begin
                    addr_def_ints = i_mean;
                    clk_counter_b = clk_counter_b + 1;
                    state = 'b11000011;
                end
            end
        end
        else
        begin
            state = 'b11000101;
        end
     end
     'b11000100:
     begin
        mean_temp_sum = Adder_Float;
        i_mean = i_mean + 1;
        state = 'b11000011;
     end
     'b11000101: 
     begin
        //mean[63:32] = Divider_Float(temp_sum, subset_size_in_ieee);
        a = mean_temp_sum;
        b = num_pxl_FP;
        state = 7'b1001100; //Divider
        temp_state = 'b11000111;
     end
     'b11000111:
     begin
        mean[63:32] = Divider_Float;
        mean[31:0] = mean_temp_sum[31:0];
        state = mean_temp_state;
     end
     'b11001000: //initial_guess
     begin
        //if(global_path_search_required) **** NOT DONE
        ig_def_img = 1'b1;
        ig_prev_u = SUBSET_DISPLACEMENT_X_FS[gid]; //****
        ig_prev_v = SUBSET_DISPLACEMENT_Y_FS[gid];
        ig_prev_t = ROTATION_Z_FS[gid];
        state = 'b11001001;
     end
     'b11001001:
     begin
        //initial_guess_4_out = initial_guess_4(def_img, prev_u, prev_v, prev_t); //****
        ig4_def_image = ig_def_img;
        ig4_u = ig_prev_u;
        ig4_v = ig_prev_v;
        ig4_t = ig_prev_t;
        state = 'b11001011; //initial_guess_4
        initial_guess_4_temp_state = 'b11001010;
     end
     'b11001010:
     begin
        initial_guess_4_out = 1'b1;
        state = initial_guess_temp_state; //end of initial_guess
     end
     'b11001011: //initial_guess_4
     begin
        // find the closes triad in the set:
        ig4_id = 32'b0;
        ig4_dist_ = 32'b0;
        //insert_motion_out = insert_motion (u, v, t); 
        //im_u = ig4_u;
        //im_v = ig4_v;
        //im_theta = ig4_t;
        //state = 'b10001000; //insert_motion
        //insert_motion_temp_state = 'b11001100;
		//aff
		imaff_u = ig4_u;
		imaff_v = ig4_v;
		imaff_theta = ig4_t;
		state = 'b1110001010; //insert_motion_aff
		insert_motion_aff_temp_state = 'b11001100;
     end
     'b11001100:
     begin
        //insert_motion_out = insert_motion;
		//aff
        //gamma = gamma_(1'b1);
        state = 'b1111100; //gamma_
        gamma_temp_state = 'b11001101; 
     end
     'b11001101:
     begin
        ig4_gamma = gamma_;
        if(ig4_gamma[31] == 1'b1)
            ig4_gamma = 32'b01000000100000000000000000000000; //4
        closest_triad_out = closest_triad(ig4_u,ig4_v,ig4_t,ig4_id,ig4_dist_); //**** 
        state = 'b11001110;
     end
     'b11001110:
     begin
        ig4_best_u = ig4_u;
        ig4_best_v = ig4_v;
        ig4_best_t = ig4_t;
        ig4_best_gamma = ig4_gamma; 
        ig4_neigh = 0; 
        state = 'b101101100; //'b11001111;//'b1101001; 
     end
     'b101101100: //'b11001111: //'b1101001:
     begin
        if(ig4_neigh < num_neighbors_)
        begin
            ig4_neigh_id = neighbor(ig4_id, ig4_neigh);
            //gamma = gamma_(1'b1);
            state = 'b1111100; //gamma_
            gamma_temp_state = 'b11001111; 
        end
        else
        begin
            state = 'b11010001;
        end
     end
     'b11001111: //'b1101010:
     begin
        ig4_gamma = gamma_;
        if(ig4_gamma[31] == 1'b1) 
            ig4_gamma = 32'b01000000100000000000000000000000;// catch a failed gamma eval
        if(less_than(ig4_gamma, ig4_best_gamma))
        begin
            ig4_best_gamma = ig4_gamma; //global
            //{best_t, best_v, best_u} = map_to_u_v_theta(centroid_x,centroid_y); //centroid_x(), centroid_y()
            //muvt_cx = centroid_x[subset_range_selection-:32];           
            //muvt_cy = centroid_y[subset_range_selection-:32];
            //muvt_cx = cx_;
            //muvt_cy = cy_;
            //state = 'b10001101; //map_to_u_v_theta 
            //map_to_u_v_theta_temp_state = 'b11010000;
			//aff
			muvt_aff_cx = cx_;
			muvt_aff_cy = cy_;
			state = 'b110010111; //map_to_u_v_theta_aff
			muvt_aff_temp_state = 'b11010000;
        end
        else
        begin
            ig4_neigh = ig4_neigh + 1;
            state = 'b101101100; //'b11001111; 
        end
     end
     'b11010000: //'b1101011:
     begin
        //ig4_best_u = map_to_u_v_theta[31:0];
        //ig4_best_v = map_to_u_v_theta[63:32];
        //ig4_best_t = map_to_u_v_theta[95:64];
        //SUBSET_DISPLACEMENT_X_FS[gid] = map_to_u_v_theta[31:0];
        //SUBSET_DISPLACEMENT_Y_FS[gid] = map_to_u_v_theta[63:32]; 
        //ROTATION_Z_FS[gid] = map_to_u_v_theta[95:64];
		//aff
		ig4_best_u = map_to_u_v_theta_aff[31:0];
        ig4_best_v = map_to_u_v_theta_aff[63:32];
        ig4_best_t = map_to_u_v_theta_aff[95:64];
        ig4_neigh = ig4_neigh + 1;
        state = 'b101101100; //'b11001111;
     end
     'b11010001: //'b1101100:
     begin
        //insert_motion_out = insert_motion(best_u,best_v,best_t);
        //im_u = ig4_best_u;
        //im_v = ig4_best_v;
        //im_theta = ig4_best_t;
        //state = 'b10001000; //insert_motion
        //insert_motion_temp_state = 'b11010010;
		//aff
		imaff_u = ig4_best_u;
		imaff_v = ig4_best_v;
		imaff_theta = ig4_best_t;
		state = 'b1110001010; //insert_motion_aff
		insert_motion_aff_temp_state = 'b11010010;
     end
     'b11010010:
     begin
        state = initial_guess_4_temp_state;
     end
     'b11010011: //'b1101110: //initialize //in SubsetSerial.cpp
     begin
        if(initialize_target == 1'b0) //maybe before function call 0 ref
        begin
                //intensities_[initialize_i] = ref_intensities_[initialize_i];
                if(clk_counter_a == 2'b10)
                begin
                    a = dout_ref_ints;
                    clk_counter_a =  2'b00;
                    if(initialize_i < num_pxl_Int) //num_pixels_
                    begin
                        initialize_i = initialize_i + 1;
                        state = 'b11010011;
                    end
                    else
                    begin
                        state = 'b11010100;
                    end                   
                end
                else
                begin
                    addr_ref_ints = initialize_i;
                    clk_counter_a = clk_counter_a + 1;
                    state = 'b11010011;
                end
        end
        else
        begin
                //intensities_[initialize_i] = def_intensities_[initialize_i];
                if(clk_counter_a == 2'b10)
                begin
                    a = dout_def_ints;
                    clk_counter_a =  2'b00;
                    if(initialize_i < num_pxl_Int) //num_pixels_
                    begin
                        initialize_i = initialize_i + 1;
                        state = 'b11010011;
                    end
                    else
                    begin
                        state = 'b11010100;
                    end                   
                end
                else
                begin
                    addr_def_ints = initialize_i;
                    clk_counter_a = clk_counter_a + 1;
                    state = 'b11010011;
                end
        end
     end
     'b11010100: //'b1101111:
     begin
        initialize_ox = offset_x;
        initialize_oy = offset_y;
        initialize_i = 0;
        state = 'b11010101; //'b1101111;
     end
     'b11010101: //'b1110000:
     begin
        if(initialize_i < num_pxl_Int) //num_pixels_
        begin
            //{mapped_y, mapped_x} = map(x_[i],y_[i],cx_,cy_);
            //map_x = x[initialize_i];
            //map_y = y[initialize_i];
            if(clk_counter_scx == 2'b10)
            begin
                //map_x = x;
				//aff
				map_aff_x = x;
                clk_counter_scx =  2'b00;
                clk_counter_scy = 2'b01;
                state = 'b11010101;                       
            end
            else if(clk_counter_scy == 2'b00)
            begin
                sub_cood_addr_x = base_address + initialize_i;
                clk_counter_scx = clk_counter_scx + 1;
                state = 'b11010101;
            end
            else if(clk_counter_scy == 2'b11)
            begin
                //map_y = y;
				//aff
				map_aff_y = y;
                clk_counter_scy =  2'b00;
                //go to the next state
                //map_cx = cx_;
				//aff
				map_aff_cx = cx_;
                //map_cy = cy_;
				//aff
				map_aff_cy = cy_;
                //state = 'b10100011; //map
				//aff
				state = 'b101111111; //map_aff
                //map_temp_state = 'b11010110;
				//aff
				map_aff_temp_state = 'b11010110;
            end
            else
            begin
                sub_cood_addr_y = base_address + initialize_i;
                clk_counter_scy = clk_counter_scy + 1;
                state = 'b11010101;
            end           
        end
        else
        begin
            state = 'b11100010; //'b1111101;
        end
     end
     'b11010110: //'b1110001:
     begin
        //initialize_mapped_x = map[31:0];
        //initialize_mapped_y = map[63:32];
		//aff
		initialize_mapped_x = map_aff[31:0];
		initialize_mapped_y = map_aff[63:32];
        if(initialize_mapped_x[23] == 1'b1) //(int_t)(mapped_x + 0.5) == (int_t)(mapped_x)
        begin
            initialize_px = initialize_mapped_x;
            state = 'b11011000;
        end
        else
        begin
            //px = Adder_Float(mapped_x, 32'b00111111100000000000000000000000);
            a = initialize_mapped_x;
            b = 32'b00111111100000000000000000000000;
            state = 7'b1000000; //Adder
            temp_state = 'b11010111;
        end
     end
     'b11010111: //'b1110010:
     begin
        initialize_px = Adder_Float;
        state = 'b11011000;
     end
     'b11011000: //'b1110011:
     begin
        if(initialize_mapped_y[23] == 1'b1) //(int_t)(mapped_x + 0.5) == (int_t)(mapped_x)
        begin
            initialize_py = initialize_mapped_y;
            state = 'b11011010;
        end
        else
        begin
            //py = Adder_Float(mapped_y, 32'b00111111100000000000000000000000);
            a = initialize_mapped_y;
            b = 32'b00111111100000000000000000000000;
            state = 7'b1000000; //Adder
            temp_state = 'b11011001;
        end
     end
     'b11011001: //'b1110100:
     begin
        initialize_py = Adder_Float;
        state = 'b11011010;
     end
     'b11011010: //'b1110101:
     begin
        if(initialize_interp == 2'b0) //BILINEAR
        begin
            //t1 = Subtractor_Float(mapped_x, ox);
            a = initialize_mapped_x;
            b = initialize_ox;
            state = 7'b1000101; //Subtractor
            temp_state = 'b11011011;
        end
        else
        begin
            initialize_i = initialize_i + 1;
            state = 'b11010101;
        end
     end
     'b11011011: //'b1110110:
     begin
        initialize_t1 = Subtractor_Float;
        //t2 = Subtractor_Float(mapped_y, oy);
        a = initialize_mapped_y;
        b = initialize_oy;
        state = 7'b1000101; //Subtractor
        temp_state = 'b11011100;
     end
     'b11011100: //'b1110111:
     begin
        initialize_t2 = Subtractor_Float;
        //intensities_[i] = interpolate_bilinear(t1,t2, width, height);
        ib_local_x = initialize_t1;
        ib_local_y = initialize_t2;
        state = 'b100000001; //interpolate_bilinear; 
        interpolate_bilinear_temp_state = 'b11011101;
     end
     'b11011101: //'b1111000:
     begin
        intensities_[i] = interpolate_bilinear;
        if(has_gradients_ == 1'b1)
        begin
            //t1 = Subtractor_Float(mapped_x, ox);
            a = initialize_mapped_x;
            b = initialize_ox;
            state = 7'b1000101; //Subtractor
            temp_state = 'b11011110;
        end
        else
        begin
            initialize_i = initialize_i + 1;
            state = 'b11010101;
        end
     end
     'b11011110: //'b1111001:
     begin
        initialize_t1 = Subtractor_Float;
        //t2 = Subtractor_Float(mapped_y, oy);
        a = initialize_mapped_y;
        b = initialize_oy;
        state = 7'b1000101; //Subtractor
        temp_state = 'b11011111;
     end
     'b11011111: //'b1111010:
     begin
        initialize_t2 = Subtractor_Float;
        //grad_x_[i] = interpolate_grad_x_bilinear(t1, t2, width, height);
        igxb_local_x = initialize_t1;
        igxb_local_y = initialize_t2;
        state = 'b100100100; //interpolate_grad_x_bilinear
        interpolate_grad_x_bilinear_temp_state = 'b11100000;
     end
     'b11100000: //'b1111011:
     begin
        grad_x[initialize_i] = interpolate_grad_x_bilinear;
        //grad_y_[i] = interpolate_grad_y_bilinear(t1, t2, width, height);
        igyb_local_x = initialize_t1;
        igyb_local_y = initialize_t2;
        state = 'b101000110; //interpolate_grad_y_bilinear
        interpolate_grad_y_bilinear_temp_state = 'b11100001;
     end
     'b11100001: //'b1111100:
     begin
        grad_y[initialize_i] = interpolate_grad_y_bilinear;
        initialize_i = initialize_i + 1;
        state = 'b11010101;
     end
     'b11100010: //'b1111101:
     begin
        if (initialize_target == 1'b0)
        begin
            if(has_gradients_ == 1'b1)
            begin
                initialize_i = 0;
                state = 'b11100011;
            end
            else
            begin
                state = 'b11100110;
            end
        end
        else
        begin
            state = 'b11100110;
        end
     end
     'b11100011: //'b1111110:
     begin
        if(initialize_i < num_pxl_Int) //num_pixels_
        begin
            //t1 = Subtractor_Float(x_[i], offset_x);
            //a = x[initialize_i];
            if(clk_counter_scx == 2'b10)
            begin
                a = x;
                clk_counter_scx =  2'b00;
                b = offset_x;
                state = 7'b1000101; //Subtractor
                temp_state = 'b11100100;                    
            end
            else
            begin
                sub_cood_addr_x = base_address + initialize_i;
                clk_counter_scx = clk_counter_scx + 1;
                state = 'b11100011;
            end          
        end
        else
        begin
            has_gradients_ = 1'b1;
            state = 'b11100110;
        end
     end
     'b11100100: //'b1111111:
     begin
        initialize_t1 = Subtractor_Float;
        //t2 = Subtractor_Float(y_[i], offset_y);
        //a = y[initialize_i];
        if(clk_counter_scy == 2'b10)
        begin
            a = y;
            clk_counter_scy =  2'b00;
            b = offset_y;
            state = 7'b1000101; //Subtractor
            temp_state = 'b11100101;                    
        end
        else
        begin
            sub_cood_addr_y = base_address+ initialize_i;
            clk_counter_scy = clk_counter_scy + 1;
            state = 'b11100100;
        end
     end
     'b11100101: //'b10000000:
     begin
        initialize_t2 = Subtractor_Float;
        //grad_x[initialize_i] = grad_x_function(t1,t2);
        //grad_y[initialize_i] = grad_y_function(t1,t2);
        if(clk_counter_a == 2'b10)
        begin
            grad_x[initialize_i] = dout_grad_x_;
            clk_counter_a =  2'b00;
            clk_counter_b = 2'b01;
            state = 'b11100101;                       
        end
        else if(clk_counter_b == 2'b00)
        begin
            addr_grad_x_ = t2 * width;
            addr_grad_x_ =  addr_grad_x_ + t1;
            clk_counter_a = clk_counter_a + 1;
            state = 'b11100101;
        end
        else if(clk_counter_b == 2'b11)
        begin
            grad_y[initialize_i] = dout_grad_y_;
            clk_counter_b =  2'b00;
            initialize_i = initialize_i + 1;
            state = 'b11100011;                    
        end
        else
        begin
            addr_grad_y_ = t2 * width;
            addr_grad_y_ =  addr_grad_y_ + t1;
            clk_counter_b = clk_counter_b + 1;
            state = 'b11100101;
        end   
     end
     'b11100110: //'b10000001:
     begin
        state = initialize_temp_state;
     end
     'b11100111: //'b10000010: //residuals
     begin
        //dx = Subtractor_Float(x, cx); 
        a = residuals_x;
        b = residuals_cx;
        state = 7'b1000101; //Subtractor
        temp_state = 'b11101000;
     end
     'b11101000: //'b10000011:
     begin
        residuals_dx = Subtractor_Float;
        //dy = Subtractor_Float(y, cy);
        a = residuals_y;
        b = residuals_cy;
        state = 7'b1000101; //Subtractor
        temp_state = 'b11101001;
     end
     'b11101001: //'b10000100:
     begin
        residuals_dy = Subtractor_Float;
        residuals_Gx = residuals_gx;
        residuals_Gy = residuals_gy;
        if(residuals_use_ref_grads == 1'b1)
        begin
            residuals_u = 32'b0;
            residuals_v = 32'b0;
            residuals_theta = 32'b0;
            //{theta, v, u} = map_to_u_v_theta(cx,cy);
            muvt_cx = residuals_cx;
            muvt_cy = residuals_cy;
            state = 'b10001101; //map_to_u_v_theta
            map_to_u_v_theta_temp_state = 'b11101010;
        end
        else
        begin
            state = 'b11110011;
        end
     end
     'b11101010: //'b10000101:
     begin
        residuals_u = map_to_u_v_theta[31:0];
        residuals_v = map_to_u_v_theta[63:32];
        residuals_theta = map_to_u_v_theta[95:64];
        SUBSET_DISPLACEMENT_X_FS[gid] = map_to_u_v_theta[31:0];
        SUBSET_DISPLACEMENT_Y_FS[gid] = map_to_u_v_theta[63:32]; 
        ROTATION_Z_FS[gid] = map_to_u_v_theta[95:64];
        //cosTheta = cos(theta);
        a = residuals_theta;
        state = 7'b1011010; //cos
        cos_temp_state = 'b11101011;
     end
     'b11101011: //'b10000110:
     begin
        residuals_cosTheta = cos;
        //sinTheta = sin(theta); 
        a = residuals_theta;
        state = 7'b1100001; //sin;
        sin_temp_state = 'b11101100;
     end
     'b11101100: //'b10000111:
     begin
        residuals_sinTheta = sin;
        //t1 = Multiplier_Float(cosTheta, gx);
        a = residuals_cosTheta;
        b = residuals_gx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11101101;
     end
     'b11101101: //'b100001000:
     begin
        residuals_t1 = Multiplier_Float;
        //t2 = Multiplier_Float(sinTheta, gy);
        a = residuals_sinTheta;
        b = residuals_gy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11101110;
     end
     'b11101110: //'b100001001:
     begin
        residuals_t2 = Multiplier_Float;
        //Gx = Subtractor_Float(t1, t2);
        a = residuals_t1;
        b = residuals_t2;
        state = 7'b1000101; //Subtractor
        temp_state = 'b11101111;
     end
     'b11101111: //'b100001010:
     begin
        residuals_Gx = Subtractor_Float;
        //t1 = Multiplier_Float(sinTheta, gx);
        a = residuals_sinTheta;
        b = residuals_gx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11110000;
     end
     'b11110000: //'b100001011:
     begin
        residuals_t1 = Multiplier_Float;
        //t2 = Multiplier_Float(cosTheta, gy);
        a = residuals_cosTheta;
        b = residuals_gy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11110001;
     end
     'b11110001: //'b100001100:
     begin
        residuals_t2 = Multiplier_Float;
        //Gy = Adder_Float(t1, t2);
        a = residuals_t1;
        b = residuals_t2;
        state = 7'b1000000; //Adder
        temp_state = 'b11110010;
     end
     'b11110010: //'b100001101:
     begin
        residuals_Gy = Adder_Float;
        state = 'b11110011;
     end
     'b11110011: //'b100001110:
     begin
        //residuals[31:0] = Multiplier_Float(Gx, dx);
        a = residuals_Gx;
        b = residuals_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11110100;
     end
     'b11110100: //'b100001111:
     begin
        residuals[31:0] = Multiplier_Float;
        //residuals[63:32] = Multiplier_Float(Gx, dy);
        a = residuals_Gx;
        b = residuals_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11110101;
     end
     'b11110101: //'b100010000:
     begin
        residuals[63:32] = Multiplier_Float;
        //t1 = Multiplier_Float(dx, dy);
        a = residuals_dx;
        b = residuals_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11110110;
     end
     'b11110110: //'b100010001:
     begin
        residuals_t1 = Multiplier_Float;
        //t2 = Multiplier_Float(dx, dx);
        a = residuals_dx;
        b = residuals_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11110111;
     end
     'b11110111: //'b100010010:
     begin
        residuals_t2 = Multiplier_Float;
        //t3 = Multiplier_Float(dy, dy); 
        a = residuals_dy;
        b = residuals_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111000;
     end
     'b11111000: //'b100010011:
     begin
        residuals_t3 = Multiplier_Float;
        //residuals[95:64] = Multiplier_Float(Gx, t1); //Gx*dx*dy;
        a = residuals_Gx;
        b = residuals_t1;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111001;
     end
     'b11111001: //'b100010100:
     begin
        residuals[95:64] = Multiplier_Float;
        //residuals[127:96] = Multiplier_Float(Gx, t2); //Gx*dx*dx;
        a = residuals_Gx;
        b = residuals_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111010;
     end
     'b11111010: //'b100010101:
     begin
        residuals[127:96] = Multiplier_Float;
        //residuals[159:128] = Multiplier_Float(Gx, t3); //Gx*dy*dy;
        a = residuals_Gx;
        b = residuals_t3;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111011;
     end
     'b11111011: //'b100010110:
     begin
        residuals[159:128] = Multiplier_Float;
        residuals[191:160] = residuals_Gx;
        //residuals[223:192] = Multiplier_Float(Gy, dx); //Gy*dx;
        a = residuals_Gy;
        b = residuals_dx;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111100;
     end
     'b11111100: //'b100010111:
     begin
        residuals[223:192] = Multiplier_Float;
        //residuals[255:224] = Multiplier_Float(Gy, dy); //Gy*dy;
        a = residuals_Gy;
        b = residuals_dy;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111101;
     end
     'b11111101: //'b100011000:
     begin
        residuals[255:224] = Multiplier_Float;
        //residuals[287:256] = Multiplier_Float(Gy, t1); //Gy*dx*dy;
        a = residuals_Gy;
        b = residuals_t1;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111110;
     end
     'b11111110: //'b100011001:
     begin
        residuals[287:256] = Multiplier_Float;
        //residuals[319:288] = Multiplier_Float(Gy, t2); //Gy*dx*dx;
        a = residuals_Gy;
        b = residuals_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b11111111;
     end
     'b11111111: //'b100011010:
     begin
        residuals[319:288] = Multiplier_Float;
        //residuals[351:320] = Multiplier_Float(Gy, t3); //Gy*dy*dy;
        a = residuals_Gy;
        b = residuals_t3;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100000000;
     end
     'b100000000: //'b100011011:
     begin
        residuals[351:320] = Multiplier_Float;
        residuals[383:352] = residuals_Gy;
        state = residuals_temp_state;
     end
     'b100000001: //'b100011100: //interpolate_bilinear
     begin
        if(ib_local_x[31] == 1'b1 || ib_local_y[31] == 1'b1)//local_x>=width_-1.5 local_y>=height_-1.5
        begin
            interpolate_bilinear = 31'b0;
            state = 'b100100011;
        end
        else
        begin
            ib_x1 = ib_local_x;
            //x2 = Adder_Float(x1,32'b00111111100000000000000000000000);
            a = ib_x1;
            b = 32'b00111111100000000000000000000000;
            state = 7'b1000000; //Adder
            temp_state = 'b100000010;
        end
     end
     'b100000010: //'b100011101:
     begin
        ib_x2 = Adder_Float;
        ib_y1 = ib_local_y;
        //y2 = Adder_Float(y1,32'b00111111100000000000000000000000);
        a = ib_y1;
        b = 32'b00111111100000000000000000000000;
        state = 7'b1000000; //Adder
        temp_state = 'b100000011;
     end
     'b100000011: //'b100011110:
     begin
        ib_y2 = Adder_Float;
        //t1 = Multiplier_Float(y1, width_);
        a = ib_y1;
        b = ib_width_;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100000100;
     end
     'b100000100: //'b100011111:
     begin
        ib_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x1);
        a = ib_t1;
        b = ib_x1;
        state = 7'b1000000; //Adder
        temp_state = 'b100000101;
     end
     'b100000101: //'b100100000:
     begin
        //t1 = Float_to_Int(t1);
        fti_input_a = ib_t1;
        state = 'b101101001; //Float_to_Int;
        Float_to_Int_temp_state = 'b100000110;
     end
     'b100000110: //'b100100001:
     begin
        ib_t1 = Float_to_Int;
        ib_t1 = intensities_[t1]; //intensities_[y1*width_+x1]
        //t2 = Subtractor_Float(x2, local_x); //(x2-local_x)
        a = ib_x2;
        b = ib_local_x;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100000111;
     end
     'b100000111: //'b100100010:
     begin
        ib_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100001000;
     end
     'b100001000: //'b100100011:
     begin
        ib_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(y2, local_y); //(y2-local_y)
        a = ib_y2;
        b = ib_local_y;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100001001;
     end
     'b100001001: //'b100100100:
     begin
        ib_t2 = Subtractor_Float;
        //t3 = Multiplier_Float(t1, t2); //intensities_[y1*width_+x1]*(x2-local_x)*(y2-local_y)
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100001010;
     end
     'b100001010: //'b100100101:
     begin
        ib_t3 = Multiplier_Float; 
        //t1 = Multiplier_Float(y1, width_);
        a = ib_y1;
        b = ib_width_;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100001011;
     end
     'b100001011: //'b100100110:
     begin
        ib_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x2);
        a = ib_t1;
        b = ib_x2;
        state = 7'b1000000; //Adder
        temp_state = 'b100001100;
     end
     'b100001100: //'b100100111:
     begin
        ib_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = ib_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100001101;
     end
     'b100001101: //'b100101000:
     begin
        ib_t1 = Float_to_Int;
        ib_t1 = intensities_[t1]; //intensities_[y1*width_+x2]
        //t2 = Subtractor_Float(local_x, x1); //(local_x-x1)
        a = ib_local_x;
        b = ib_x1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100001110;
     end
     'b100001110: //'b100101001:
     begin
        ib_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100001111;
     end
     'b100001111: //'b100101010:
     begin
        ib_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(y2, local_y); //(y2-local_y)
        a = ib_y2;
        b = ib_local_y;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100010000;
     end
     'b100010000: //'b100101011:
     begin
        ib_t2 = Subtractor_Float;
        //t4 = Multiplier_Float(t1, t2); //intensities_[y1*width_+x2]*(local_x-x1)*(y2-local_y)
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100010001;
     end
     'b100010001: //'b100101100:
     begin
        ib_t4 = Multiplier_Float;
        //t1 = Multiplier_Float(y2, width_);
        a = ib_y2;
        b = ib_width_;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100010010;
     end
     'b100010010: //'b100101101:
     begin
        ib_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x2);
        a = ib_t1;
        b = ib_x2;
        state = 7'b1000000; //Adder
        temp_state = 'b100010011;
     end
     'b100010011: //'b100101110:
     begin
        ib_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = ib_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100010100;
     end
     'b100010100: //'b100101111:
     begin
        ib_t1 = Float_to_Int;
        ib_t1 = intensities_[t1]; //intensities_[y2*width_+x2]
        //t2 = Subtractor_Float(local_x, x1); //(local_x-x1)
        a = ib_local_x;
        b = ib_x1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100010101;
     end
     'b100010101: //'b100110000:
     begin
        ib_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100010110;
     end
     'b100010110: //'b100110001:
     begin
        ib_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(local_y, y1); //(local_y-y1)
        a = ib_local_y;
        b = ib_y1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100010111;
     end
     'b100010111: //'b100110010:
     begin
        ib_t2 = Subtractor_Float;
        //t5 = Multiplier_Float(t1, t2); //intensities_[y2*width_+x2]*(local_x-x1)*(local_y-y1) 
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100011000;
     end
     'b100011000: //'b100110011:
     begin
        ib_t5 = Multiplier_Float;
        //t1 = Multiplier_Float(y2, width_);
        a = ib_y2;
        b = ib_width_;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100011001; 
     end
     'b100011001: //'b100110100:
     begin
        ib_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x1);
        a = ib_t1;
        b = ib_x1;
        state = 7'b1000000; //Adder
        temp_state = 'b100011010; 
     end
     'b100011010: //'b100110101:
     begin
        ib_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = ib_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100011011;
     end
     'b100011011: //'b100110110:
     begin
        ib_t1 = Float_to_Int;
        ib_t1 = intensities_[t1]; //intensities_[y2*width_+x1]
        //t2 = Subtractor_Float(x2, local_x); //(x2-local_x)
        a = ib_x2;
        b = ib_local_x;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100011100; 
     end
     'b100011100: //'b100110111:
     begin
        ib_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100011101; 
     end
     'b100011101: //'b100111000:
     begin
        ib_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(local_y, y1); //(local_y-y1)
        a = ib_local_y;
        b = ib_y1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100011110; 
     end
     'b100011110: //'b100111001:
     begin
        ib_t2 = Subtractor_Float;
        //t6 = Multiplier_Float(t1, t2); //intensities_[y2*width_+x1]*(x2-local_x)*(local_y-y1) 
        a = ib_t1;
        b = ib_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100011111;
     end
     'b100011111: //'b100111010:
     begin
        ib_t6 = Multiplier_Float;
        //t3 = Adder_Float(t3, t4);
        a = ib_t3;
        b = ib_t4;
        state = 7'b1000000; //Adder
        temp_state = 'b100100000;
     end
     'b100100000: //'b100111011:
     begin
        ib_t3 = Adder_Float;
        //t3 = Adder_Float(t3, t5);
        a = ib_t3;
        b = ib_t5;
        state = 7'b1000000; //Adder
        temp_state = 'b100100001;
     end
     'b100100001: //'b100111100:
     begin
        ib_t3 = Adder_Float;
        //interpolate_bilinear = Adder_Float(t3, t6);    
        a = ib_t3;
        b = ib_t6;
        state = 7'b1000000; //Adder
        temp_state = 'b100100010;
     end
     'b100100010: //'b100111101:
     begin
        interpolate_bilinear = Adder_Float;
        state = 'b100100011;
     end
     'b100100011: //'b100111110:
     begin
        state = interpolate_bilinear_temp_state;
     end
     'b100100100: //'b100111111: //interpolate_grad_x_bilinear 
     begin
        if(igxb_local_x[31] == 1'b1 || igxb_local_y[31] == 1'b1)//local_x>=width_-1.5 local_y>=height_-1.5
        begin
            interpolate_grad_x_bilinear = 32'b0;
            state = 'b101000101;
        end
        else
        begin
            igxb_x1 = igxb_local_x;
            //x2 = Adder_Float(x1,32'b00111111100000000000000000000000);
            a = igxb_x1;
            b = 32'b00111111100000000000000000000000;
            state = 7'b1000000; //Adder
            temp_state = 'b100100101;
        end
     end
     'b100100101: //'b100111111:
     begin
        igxb_x2 = Adder_Float;
        igxb_y1 = igxb_local_y;
        //y2 = Adder_Float(y1,32'b00111111100000000000000000000000);
        a = igxb_y1;
        b = 32'b00111111100000000000000000000000;
        state = 7'b1000000; //Adder
        temp_state = 'b100100110;
     end
     'b100100110: //'b101000000:
     begin
        igxb_y2 = Adder_Float;
        //t1 = Multiplier_Float(y1, width_in);
        a = igxb_y1;
        b = igxb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100100111;
     end
     'b100100111: //'b101000001:
     begin
        igxb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x1);
        a = igxb_t1;
        b = igxb_x1;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100101000;
     end
     'b100101000: //'b101000010:
     begin
        igxb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igxb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100101001;
     end
     'b100101001: //'b101000011:
     begin
        igxb_t1 = Float_to_Int;
        //igxb_t1 = grad_x_[igxb_t1]; //intensities_[y1*width_+x1]
        //t2 = Subtractor_Float(x2, local_x); //(x2-local_x)
        if(clk_counter_a == 2'b10)
        begin
            igxb_t1 = dout_grad_x_;
            clk_counter_a =  2'b00;
            a = igxb_x2;
            b = igxb_local_x;
            state = 7'b1000101; //Subtractor
            temp_state = 'b100101010;                    
        end
        else
        begin
            addr_grad_x_ = igxb_t1;
            clk_counter_a = clk_counter_a + 1;
            state = 'b100101001;
        end 
     end
     'b100101010: //'b101000100:
     begin
        igxb_t2 = Subtractor_Float;
        //t2 = Subtractor_Float(y2, local_y); //(y2-local_y)
        a = igxb_y2;
        b = igxb_local_y;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100101011;
     end
     'b100101011: //'b101000101:
     begin
        igxb_t2 = Subtractor_Float;
        //t3 = Multiplier_Float(t1, t2); //intensities_[y1*width_+x1]*(x2-local_x)*(y2-local_y)
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100101100;
     end
     'b100101100: //'b101000110:
     begin
         igxb_t3 = Multiplier_Float;
         //t1 = Multiplier_Float(y1, width_in);
         a = igxb_y1;
         b = igxb_width_in;
         state = 7'b1000110; //Multiplier
         temp_state = 'b100101101;  
     end
     'b100101101: //'b101000111:
     begin
        igxb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x2);
        a = igxb_t1;
        b = igxb_x2;
        state = 7'b1000000; //Adder
        temp_state = 'b100101110;
     end
     'b100101110: //'b101001000:
     begin
        igxb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igxb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100101111; 
     end
     'b100101111: //'b101001001:
     begin
        igxb_t1 = Float_to_Int;
        //igxb_t1 = grad_x_[igxb_t1]; //intensities_[y1*width_+x2]
        //t2 = Subtractor_Float(local_x, x1); //(local_x-x1)       
        if(clk_counter_a == 2'b10)
        begin
            igxb_t1 = dout_grad_x_;
            clk_counter_a =  2'b00;
            a = igxb_local_x;
            b = igxb_x1;
            state = 7'b1000101; //Subtractor
            temp_state = 'b100110000;                    
        end
        else
        begin
            addr_grad_x_ = igxb_t1;
            clk_counter_a = clk_counter_a + 1;
            state = 'b100101111;
        end 
     end
     'b100110000: //'b101001010:
     begin
        igxb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100110001;
     end
     'b100110001: //'b101001011:
     begin
        igxb_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(y2, local_y); //(y2-local_y)
        a = igxb_y2;
        b = igxb_local_y;
        state = 7'b1000101; //Subtractor
        temp_state = 'b100110010;
     end
     'b100110010: //'b101001100:
     begin
        igxb_t2 = Subtractor_Float;
        //t4 = Multiplier_Float(t1, t2); //intensities_[y1*width_+x2]*(local_x-x1)*(y2-local_y)
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100110011;
     end
     'b100110011: //'b101001101:
     begin
        igxb_t4 = Multiplier_Float;
        //t1 = Multiplier_Float(y2, width_in);
        a = igxb_y2;
        b = igxb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100110100;
     end
     'b100110100: //'b101001110:
     begin
        igxb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x2);
        a = igxb_t1;
        b = igxb_x2;
        state = 7'b1000000; //Adder
        temp_state = 'b100110101;
     end
     'b100110101: //'b101001111:
     begin
        igxb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igxb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100110110;
     end
     'b100110110: //'b101010000:
     begin
        igxb_t1 = Float_to_Int;
        //igxb_t1 = grad_x_[igxb_t1]; //intensities_[y2*width_+x2]
        //t2 = Subtractor_Float(local_x, x1); //(local_x-x1)
        if(clk_counter_a == 2'b10)
        begin
            igxb_t1 = dout_grad_x_;
            clk_counter_a =  2'b00;
            a = igxb_local_x;
            b = igxb_x1;
            state = 7'b1000101; //Subtractor
            temp_state = 'b100110111;                    
        end
        else
        begin
            addr_grad_x_ = igxb_t1;
            clk_counter_a = clk_counter_a + 1;
            state = 'b100110110;
        end 
     end
     'b100110111: //'b101010001:
     begin
        igxb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100111000;
     end
     'b100111000: //'b101010010:
     begin
         igxb_t1 = Multiplier_Float;
         //t2 = Subtractor_Float(local_y, y1); //(local_y-y1)
         a = igxb_local_y;
         b = igxb_y1;
         state = 7'b1000101; //Subtractor
         temp_state = 'b100111001; 
     end
     'b100111001: //'b101010011:
     begin
        igxb_t2 = Subtractor_Float;
        //t5 = Multiplier_Float(t1, t2); //intensities_[y2*width_+x2]*(local_x-x1)*(local_y-y1)      
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100111010; 
     end
     'b100111010: //'b101010100:
     begin
        igxb_t5 = Multiplier_Float;
        //t1 = Multiplier_Float(y2, width_in);
        a = igxb_y2;
        b = igxb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100111011; 
     end
     'b100111011: //'b101010101:
     begin
        igxb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x1);
        a = igxb_t1;
        b = igxb_x1;
        state = 7'b1000000; //Adder
        temp_state = 'b100111100; 
     end
     'b100111100: //'b101010110:
     begin
        igxb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igxb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b100111101;
     end
     'b100111101: //'b101010111:
     begin
        //igxb_t1 = grad_x_[igxb_t1]; //intensities_[y2*width_+x1]
        //t2 = Subtractor_Float(x2, local_x); //(x2-local_x)
        if(clk_counter_a == 2'b10)
        begin
            igxb_t1 = dout_grad_x_;
            clk_counter_a =  2'b00;
            a = igxb_x2;
            b = igxb_local_x;
            state = 7'b1000101; //Subtractor
            temp_state = 'b100111110;                   
        end
        else
        begin
            addr_grad_x_ = igxb_t1;
            clk_counter_a = clk_counter_a + 1;
            state = 'b100111101;
        end  
     end
     'b100111110: //'b101011000:
     begin
        igxb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b100111111; 
     end
     'b100111111: //'b101011001:
     begin
        igxb_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(local_y, y1); //(local_y-y1)
        a = igxb_local_y;
        b = igxb_y1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b101000000;
     end
     'b101000000: //'b101011010:
     begin
        igxb_t2 = Subtractor_Float;
        //t6 = Multiplier_Float(t1, t2); //intensities_[y2*width_+x1]*(x2-local_x)*(local_y-y1)
        a = igxb_t1;
        b = igxb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101000001;
     end
     'b101000001: //'b101011011:
     begin
        igxb_t6 = Multiplier_Float;
        //t3 = Adder_Float(t3, t4);
        a = igxb_t3;
        b = igxb_t4;
        state = 7'b1000000; //Adder
        temp_state = 'b101000010;
     end
     'b101000010: //'b101011100:
     begin
        igxb_t3 = Adder_Float;
        //t3 = Adder_Float(t3, t5);
        a = igxb_t3;
        b = igxb_t5;
        state = 7'b1000000; //Adder
        temp_state = 'b101000011;
     end
     'b101000011: //'b101011101:
     begin
        igxb_t3 = Adder_Float;
        //interpolate_grad_x_bilinear = Adder_Float(t3, t6);    
        a = igxb_t3;
        b = igxb_t6;
        state = 7'b1000000; //Adder
        temp_state = 'b101000100;
     end
     'b101000100: //'b101011110:
     begin
        interpolate_grad_x_bilinear = Adder_Float;
        state = 'b101000101; 
     end
     'b101000101: //'b101011111:
     begin
        state = interpolate_grad_x_bilinear_temp_state;
     end
     'b101000110: //'b101100000: //interpolate_grad_y_bilinear
     begin
        if(igyb_local_x[31] == 1'b1 || igyb_local_y[31] == 1'b1)//local_x>=width_-1.5 local_y>=height_-1.5
        begin
            interpolate_grad_y_bilinear = 31'b0;
            state = 'b101101000;
        end
        else
        begin
            igyb_x1 = igyb_local_x;
            //x2 = Adder_Float(x1,32'b00111111100000000000000000000000);
            a = igyb_x1;
            b = 32'b00111111100000000000000000000000;
            state = 7'b1000000; //Adder
            temp_state = 'b101000111;
        end
     end
     'b101000111: //'b101100001:
     begin
        igyb_x2 = Adder_Float;
        igyb_y1 = igyb_local_y;
        //y2 = Adder_Float(y1,32'b00111111100000000000000000000000);
        a = igyb_y1;
        b = 32'b00111111100000000000000000000000;
        state = 7'b1000000; //Adder
        temp_state = 'b101001000;
     end
     'b101001000: //'b101100010:
     begin
        igyb_y2 = Adder_Float;
        //t1 = Multiplier_Float(y1, width_in);
        a = igyb_y1;
        b = igyb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101001001;
     end
     'b101001001: //'b101100011:
     begin
        igyb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x1);
        a = igyb_t1;
        b = igyb_x1;
        state = 7'b1000000; //Adder
        temp_state = 'b101001010;
     end
     'b101001010: //'b101100100:
     begin
        igyb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igyb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b101001011;
     end
     'b101001011: //'b101100101:
     begin
        igyb_t1 = Float_to_Int;
        //igyb_t1 = grad_y_[igyb_t1]; //intensities_[y1*width_+x1]
        //t2 = Subtractor_Float(x2, local_x); //(x2-local_x)
        if(clk_counter_b == 2'b10)
        begin
            igyb_t1 = dout_grad_y_;
            clk_counter_b =  2'b00;
            a = igyb_x2;
            b = igyb_local_x;
            state = 7'b1000101; //Subtractor
            temp_state = 'b101001100;                   
        end
        else
        begin
            addr_grad_y_ = igyb_t1;
            clk_counter_b = clk_counter_b + 1;
            state = 'b101001011;
        end
     end
     'b101001100: //'b101100110:
     begin
        igyb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101001101;
     end
     'b101001101: //'b101100111:
     begin
        igyb_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(y2, local_y); //(y2-local_y)
        a = igyb_y2;
        b = igyb_local_y;
        state = 7'b1000101; //Subtractor
        temp_state = 'b101001110;
     end
     'b101001110: //'b101101000:
     begin
        igyb_t2 = Subtractor_Float;
        //t3 = Multiplier_Float(t1, t2); //intensities_[y1*width_+x1]*(x2-local_x)*(y2-local_y)
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101001111;
     end
     'b101001111: //'b101101001:
     begin
        igyb_t3 = Multiplier_Float;
        //t1 = Multiplier_Float(y1, width_in);
        a = igyb_y1;
        b = igyb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101010000;
     end
     'b101010000: //'b101101010:
     begin
        igyb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x2);
        a = igyb_t1;
        b = igyb_x2;
        state = 7'b1000000; //Adder
        temp_state = 'b101010001;
     end
     'b101010001: //'b101101011:
     begin
        igyb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igyb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b101010010;
     end
     'b101010010: //'b101101100:
     begin
        igyb_t1 = Float_to_Int;
        //igyb_t1 = grad_y_[igyb_t1]; //intensities_[y1*width_+x2]
        //t2 = Subtractor_Float(local_x, x1); //(local_x-x1)
        if(clk_counter_b == 2'b10)
        begin
            igyb_t1 = dout_grad_y_;
            clk_counter_b =  2'b00;
            a = igyb_local_x;
            b = igyb_x1;
            state = 7'b1000101; //Subtractor
            temp_state = 'b101010011;                  
        end
        else
        begin
            addr_grad_y_ = igyb_t1;
            clk_counter_b = clk_counter_b + 1;
            state = 'b101010010;
        end
     end
     'b101010011: //'b101101101:
     begin
        igyb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a =igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101010100;
     end
     'b101010100: //'b101101110:
     begin
        igyb_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(y2, local_y); //(y2-local_y)
        a = igyb_y2;
        b = igyb_local_y;
        state = 7'b1000101; //Subtractor
        temp_state = 'b101010101;
     end
     'b101010101: //'b101101111:
     begin
        igyb_t2 = Subtractor_Float;
        //t4 = Multiplier_Float(t1, t2); //intensities_[y1*width_+x2]*(local_x-x1)*(y2-local_y)
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101010110;
     end
     'b101010110:
     begin
        igyb_t4 = Multiplier_Float;
        //t1 = Multiplier_Float(y2, width_in);
        a = igyb_y2;
        b = igyb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101010111;
     end
     'b101010111: //'b101110001:
     begin
        igyb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x2);
        a = igyb_t1;
        b = igyb_x2;
        state = 7'b1000000; //Adder
        temp_state = 'b101011000;
     end
     'b101011000:
     begin
        igyb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igyb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b101011001;
     end
     'b101011001: //'b101110011:
     begin
        igyb_t1 = Float_to_Int;
        //igyb_t1 = grad_y_[igyb_t1]; //intensities_[y2*width_+x2]
        //t2 = Subtractor_Float(local_x, x1); //(local_x-x1)
        if(clk_counter_b == 2'b10)
        begin
            igyb_t1 = dout_grad_y_;
            clk_counter_b =  2'b00;
            a = igyb_local_x;
            b = igyb_x1;
            state = 7'b1000101; //Subtractor
            temp_state = 'b101011010;                
        end
        else
        begin
            addr_grad_y_ = igyb_t1;
            clk_counter_b = clk_counter_b + 1;
            state = 'b101011001;
        end
     end
     'b101011010: //'b101110100:
     begin
        igyb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101011011;
     end
     'b101011011:
     begin
        igyb_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(local_y, y1); //(local_y-y1)
        a = igyb_local_y;
        b = igyb_y1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b101011100;
     end
     'b101011100:
     begin
        igyb_t2 = Subtractor_Float;
        //t5 = Multiplier_Float(t1, t2); //intensities_[y2*width_+x2]*(local_x-x1)*(local_y-y1) 
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101011101;
     end
     'b101011101:
     begin
        igyb_t5 = Multiplier_Float;
        //t1 = Multiplier_Float(y2, width_in);
        a = igyb_y2;
        b = igyb_width_in;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101011110;
     end
     'b101011110:
     begin
        igyb_t1 = Multiplier_Float;
        //t1 = Adder_Float(t1, x1);
        a = igyb_t1;
        b = igyb_x1;
        state = 7'b1000000; //Adder
        temp_state = 'b101011111;
     end
     'b101011111:
     begin
        igyb_t1 = Adder_Float;
        //t1 = Float_to_Int(t1);
        fti_input_a = igyb_t1;
        state = 'b101101001; //Float_to_Int
        Float_to_Int_temp_state = 'b101100000;
     end
     'b101100000:
     begin
        igyb_t1 = Float_to_Int;
        //igyb_t1 = grad_y_[igyb_t1]; //intensities_[y2*width_+x1]
        //t2 = Subtractor_Float(x2, local_x); //(x2-local_x)
        if(clk_counter_b == 2'b10)
        begin
            igyb_t1 = dout_grad_y_;
            clk_counter_b =  2'b00;
            a = igyb_x2;
            b = igyb_local_x;
            state = 7'b1000101; //Subtractor
            temp_state = 'b101100001;              
        end
        else
        begin
            addr_grad_y_ = igyb_t1;
            clk_counter_b = clk_counter_b + 1;
            state = 'b101100000;
        end
     end
     'b101100001:
     begin
        igyb_t2 = Subtractor_Float;
        //t1 = Multiplier_Float(t1, t2);
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101100010;
     end
     'b101100010:
     begin
        igyb_t1 = Multiplier_Float;
        //t2 = Subtractor_Float(local_y, y1); //(local_y-y1)
        a = igyb_local_y;
        b = igyb_y1;
        state = 7'b1000101; //Subtractor
        temp_state = 'b101100011;
     end
     'b101100011:
     begin
        igyb_t2 = Subtractor_Float;
        //t6 = Multiplier_Float(t1, t2); //intensities_[y2*width_+x1]*(x2-local_x)*(local_y-y1) 
        a = igyb_t1;
        b = igyb_t2;
        state = 7'b1000110; //Multiplier
        temp_state = 'b101100100;
     end
     'b101100100:
     begin
        igyb_t6 = Multiplier_Float;
        //t3 = Adder_Float(t3, t4);
        a = igyb_t3;
        b = igyb_t4;
        state = 7'b1000000; //Adder
        temp_state = 'b101100101;
     end
     'b101100101:
     begin
        igyb_t3 = Adder_Float;
        //t3 = Adder_Float(t3, t5);
        a = igyb_t3;
        b = igyb_t5;
        state = 7'b1000000; //Adder
        temp_state = 'b101100110;
     end
     'b101100110:
     begin
        igyb_t3 = Adder_Float;
        //interpolate_grad_y_bilinear = Adder_Float(t3, t6);   
        a = igyb_t3;
        b = igyb_t6;
        state = 7'b1000000; //Adder
        temp_state = 'b101100111;
     end
     'b101100111: //'b110000001:
     begin
        interpolate_grad_y_bilinear = Adder_Float;
        state = 'b101101000;
     end
     'b101101000: //'b110000010:
     begin
        state = interpolate_grad_y_bilinear_temp_state;
     end
     'b101101001: //'b110000010: //Float_to_Int
     begin
        fti_a = fti_input_a;
        fti_a_m[31:8] = {1'b1, fti_a[22 : 0]};
        fti_a_m[7:0] = 0;
        fti_a_e = fti_a[30 : 23] - 127;
        fti_a_s = fti_a[31];
        if ($signed(fti_a_e) == -127) begin
            fti_z = 0;
            fti_s_output_z = fti_z;
            state = 'b101101011;
        end 
        else if ($signed(fti_a_e) > 31) 
        begin
            fti_z = 32'h80000000;
            fti_s_output_z = fti_z;
            state = 'b101101011;
        end 
        else 
        begin
            fti_i = 0;
            state = 'b101101010;
        end
     end
     'b101101010:
     begin
        if( fti_i < 32)
        begin
            if ($signed(fti_a_e) < 31 && fti_a_m) 
            begin
                fti_a_e = fti_a_e + 1;
                fti_a_m = fti_a_m >> 1;
            end 
            else 
            begin
                if (fti_a_m[31]) 
                begin
                    fti_z = 32'h80000000;
                end 
                else 
                begin
                    fti_z = fti_a_s ? -fti_a_m : fti_a_m;
                end
                fti_s_output_z = fti_z;
            end
            fti_i = fti_i + 1;
            state = 'b101101010;
        end
        else
        begin
            state = 'b101101011;
        end
     end
     'b101101011: //'b110000100:
     begin
        Float_to_Int = fti_s_output_z;
        state = Float_to_Int_temp_state;
     end
	 'b101110101: //Update //Upd
	 begin
	   l = 0;
	   state = 'b101111100;
	 end
	 'b101111100:
	 begin
	   if(l <= N)
	   begin
	       a = parameters_[l];
	       b = def_update[l];
	       state = 7'b1000000; //Adder
	       temp_state = 'b101111101;
	   end
	   else
	   begin
	       state = 'b101111110;
	   end
	 end
	 'b101111101:
	 begin
	    parameters_[l] = Adder_Float;
	    l = l + 1;
	    state = 'b101111100;
	 end
	 'b101111110:
	 begin
	   state = update_temp_state;
	 end
	 'b101110110: //test_for_convergence
	 begin
		converged = 1'b1;
		p = 0;
		state = 'b101111000;	
	 end
	 'b101111000:
	 begin
		if(p <= N)
		begin
			//if(std::abs(parameters_[i] - old_parameters[i]) >= tol)
			a = residuals_out[p*32+31-:32];
			b = def_old[p];
			state = 7'b1000101; //Subtractor
			temp_state = 'b101110111;
		end
		else
		begin
			state = 'b101111001;
		end
	 end
	 'b101110111:
	 begin
		if(less_than(fast_solver_tolerance, Subtractor_Float))
		begin
			//converged = false;
			converged = 1'b0;
		end
		else
		begin
		   converged = 1'b1;
		end
		p = p + 1;
		state = 'b101111000;
	 end
	 'b101111001:
	 begin
		state = test_for_convergence_temp_state;
	 end
	 'b101111111: //map_aff
	 begin
		map_aff_dx = 32'b0;
		map_aff_dy = 32'b0;
		map_aff_Dx = 32'b0;
		map_aff_Dy = 32'b0;
		a = parameters_[2]; //ROTATION_Z_FS[gid];
		state = 'b1011010; //cos
		cos_temp_state = 'b110000000;
	 end
	 'b110000000:
	 begin
		map_aff_cost = cos;
		a = parameters_[2]; //ROTATION_Z_FS[gid];
		state = 'b1100001; //sin
		sin_temp_state = 'b110000001;
	 end
	 'b110000001:
	 begin
		map_aff_sint = sin;
		a = map_aff_x;
		b = map_aff_cx;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110000010;
	 end
	 'b110000010:
	 begin
		map_aff_dx = Subtractor_Float;
		a = map_aff_y;
		b = map_aff_cy;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110000011;
	 end
	 'b110000011:
	 begin
		map_aff_dy = Subtractor_Float;
		a = parameters_[3]; //NORMAL_STRETCH_XX_FS[gid];
		b = 32'b00111111100000000000000000000000; //1
		state = 7'b1000000; //Adder
		temp_state = 'b1110100010;
	 end
	 'b1110100010:
	 begin
	    a = Adder_Float;
	    b = map_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110000101;
	 end
	 'b110000101:
	 begin
		map_aff_t1 = Multiplier_Float; //(1.0+parameter(NORMAL_STRETCH_XX_FS))*dx
		a = parameters_[5]; //SHEAR_STRETCH_XY_FS[gid];
		b = map_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110000110;
	 end
	 'b110000110:
	 begin
		map_aff_t2 = Multiplier_Float; //parameter(SHEAR_STRETCH_XY_FS)*dy
		a = map_aff_t1;
		b = map_aff_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110000111;
	 end
	 'b110000111:
	 begin
		map_aff_Dx = Adder_Float;
		a = parameters_[4]; //NORMAL_STRETCH_YY_FS[gid];
		b = 32'b00111111100000000000000000000000; //1
		state = 7'b1000000; //Adder
		temp_state = 'b110001000;
	 end
	 'b110001000:
	 begin
		a = Adder_Float;
		b = map_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110001001;
	 end
	 'b110001001:
	 begin
		map_aff_t1 = Multiplier_Float; //(1.0+parameter(NORMAL_STRETCH_YY_FS))*dy
		a = parameters_[5]; //SHEAR_STRETCH_XY_FS[gid];
		b = map_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110001010;
	 end
	 'b110001010:
	 begin
		map_aff_t2 = Multiplier_Float; //parameter(SHEAR_STRETCH_XY_FS)*dx
		a = map_aff_t1;
		b = map_aff_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110001100;
	 end
	 'b110001100:
	 begin
		map_aff_Dy = Adder_Float;
		a = map_aff_cost;
		b = map_aff_Dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110001101;
	 end
	 'b110001101:
	 begin
		map_aff_t1 = Multiplier_Float;
		a = map_aff_sint;
		b = map_aff_Dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110001110;
	 end
	 'b110001110:
	 begin
		map_aff_t2 = Multiplier_Float;
		a = map_aff_t1;
		b = map_aff_t2;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110001111;
	 end
	 'b110001111:
	 begin
		map_aff_t3 = Subtractor_Float;
		a = parameters_[0]; //SUBSET_DISPLACEMENT_X_FS[gid];
		b = map_aff_t3;
		state = 7'b1000000; //Adder
		temp_state = 'b110010000;
	 end
	 'b110010000:
	 begin
		a = Adder_Float;
		b = map_aff_cx;
		state = 7'b1000000; //Adder
		temp_state = 'b110010001;
	 end
	 'b110010001:
	 begin
		map_aff_out_x = Adder_Float;
		a = map_aff_sint;
		b = map_aff_Dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110010010;
	 end
	 'b110010010:
	 begin
		map_aff_t1 = Multiplier_Float;
		a = map_aff_cost;
		b = map_aff_Dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110010011;
	 end
	 'b110010011:
	 begin
		map_aff_t2 = Multiplier_Float;
		a = map_aff_t1;
		b = map_aff_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110010100;
	 end
	 'b110010100:
	 begin
		a = Adder_Float;
		b = parameters_[1]; //SUBSET_DISPLACEMENT_Y_FS[gid];
		state = 7'b1000000; //Adder
		temp_state = 'b110010101;
	 end
	 'b110010101:
	 begin
		a = Adder_Float;
		b = map_aff_cy;
		state = 7'b1000000; //Adder
		temp_state = 'b110010110;
	 end
	 'b110010110:
	 begin
		map_aff_out_y = Adder_Float;
		map_aff[31:0] = map_aff_out_x;
		map_aff[63:32] = map_aff_out_y;
		state = map_aff_temp_state;
	 end
	 'b110010111: //map_to_u_v_theta_aff
	 begin
		map_to_u_v_theta_aff[31:0] = parameters_[0]; //SUBSET_DISPLACEMENT_X_FS[gid];
		map_to_u_v_theta_aff[63:32] = parameters_[1]; //SUBSET_DISPLACEMENT_Y_FS[gid];
		map_to_u_v_theta_aff[95:64] = parameters_[2];//ROTATION_Z_FS[gid];
		state = muvt_aff_temp_state;
	 end	 
	'b1110010000: //residuals_aff
	begin
        residuals_aff_dx = 32'b0;
        residuals_aff_dy = 32'b0;
        residuals_aff_Dx = 32'b0; 
        residuals_aff_Dy = 32'b0;
        residuals_aff_delTheta = 32'b0;
        residuals_aff_delEx = 32'b0;
        residuals_aff_delEy = 32'b0;
        residuals_aff_delGxy = 32'b0;
        residuals_aff_Gx = 32'b0;
        residuals_aff_Gy = 32'b0;
        residuals_aff_theta = 32'b0;
        residuals_aff_dudx = 32'b0;
        residuals_aff_dvdy = 32'b0;
        residuals_aff_gxy = 32'b0;
        residuals_aff_cosTheta = 32'b0;
        residuals_aff_sinTheta = 32'b0;
        residulas_t1 = 32'b0;
        residulas_t2 = 32'b0;
        residulas_t3 = 32'b0;
        residulas_t4 = 32'b0;
        residulas_t5 = 32'b0;
        residulas_t6 = 32'b0;
        residuals_aff = 192'b0;
		residuals_aff_theta = parameters_[2]; //ROTATION_Z_FS[gid];
		residuals_aff_dudx = parameters_[3]; //NORMAL_STRETCH_XX_FS[gid];
		residuals_aff_dvdy = parameters_[4]; //NORMAL_STRETCH_YY_FS[gid];
		residuals_aff_gxy = parameters_[5]; //SHEAR_STRETCH_XY_FS[gid];
		a = residuals_aff_theta;
		state = 'b1011010; //cos
		cos_temp_state = 'b110011000;
	end
	'b110011000:
	begin
		residuals_aff_cosTheta = cos;
		a = residuals_aff_theta;
		state = 'b1100001; //sin
		sin_temp_state = 'b110011001;
	end
	'b110011001:
	begin
		residuals_aff_sinTheta = sin;
		a = residuals_aff_x;
		b = residuals_aff_cx;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110011010;
	end
	'b110011010:
	begin
		residuals_aff_dx = Subtractor_Float;
		a = residuals_aff_y;
		b = residuals_aff_cy;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110011011;
	end
	'b110011011:
	begin
		residuals_aff_dy = Subtractor_Float;
		a = 32'b00111111100000000000000000000000; //1
		b = residuals_aff_dudx;
		state = 7'b1000000; //Adder
		temp_state = 'b110011100;
	end
	'b110011100:
	begin
		a = Adder_Float;
		b = residuals_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110011101;
	end
	'b110011101:
	begin
		residulas_t1 = Multiplier_Float;
		a = residuals_aff_gxy;
		b = residuals_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110011110;
	end
	'b110011110:
	begin
		a = Multiplier_Float;
		b = residulas_t1;
		state = 7'b1000000; //Adder
		temp_state = 'b110011111;
	end
	'b110011111:
	begin
		residuals_aff_Dx = Adder_Float;
		a = 32'b00111111100000000000000000000000; //1
		b = residuals_aff_dvdy;
		state = 7'b1000000; //Adder
		temp_state = 'b110100000;
	end
	'b110100000:
	begin
		a = Adder_Float;
		b = residuals_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110100001;
	end
	'b110100001:
	begin
		residulas_t2 = Multiplier_Float;
		a = residuals_aff_gxy;
		b = residuals_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110100010;
	end
	'b110100010:
	begin
		a = Multiplier_Float;
		b = residulas_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110100011;
	end
	'b110100011:
	begin
		residuals_aff_Dy = Adder_Float;
		if(residuals_aff_use_ref_grads == 1'b1)
		begin
			a = residuals_aff_cosTheta;
			b = residuals_aff_gx;
			state = 7'b1000110; //Multiplier
			temp_state = 'b110100100;
		end
		else
		begin
			residuals_aff_Gx = residuals_aff_gx;
			residuals_aff_Gy = residuals_aff_gy;
			state = 'b110101010;
		end
	end
	'b110100100:
	begin
		residulas_t1 = Multiplier_Float;
		a = residuals_aff_sinTheta;
		b = residuals_aff_gy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110100101;
	end
	'b110100101:
	begin
		residulas_t2 = Multiplier_Float;
		a = residulas_t1;
		b = residulas_t2;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110100110;
	end
	'b110100110:
	begin
		residuals_aff_Gx = Subtractor_Float;
		a = residuals_aff_sinTheta;
		b = residuals_aff_gx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110100111;
	end
	'b110100111:
	begin
		residulas_t1 = Multiplier_Float;
		a = residuals_aff_cosTheta;
		b = residuals_aff_gy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110101000;
	end
	'b110101000:
	begin
		residulas_t2 = Multiplier_Float;
		a = residulas_t1;
		b = residulas_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110101001;
	end
	'b110101001:
	begin
		residuals_aff_Gy = Adder_Float;
		state = 'b110101010;
	end
	'b110101010:
	begin
		a[31] = 1'b1;
		a[30:0] = residuals_aff_sinTheta[30:0];
		b = residuals_aff_Dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110101011;
	end
	'b110101011:
	begin
		residulas_t1 = Multiplier_Float;
		a[31] = 1'b1;
		a[30:0] = residuals_aff_cosTheta[30:0];
		b = residuals_aff_Dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110101100;
	end
	'b110101100:
	begin
		residulas_t2 = Multiplier_Float;
		a = residulas_t1;
		b = residulas_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110101101;
	end
	'b110101101:
	begin
		residulas_t3 = Adder_Float;
		a = residuals_aff_Gx;
		b = residulas_t3;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110101110;
	end
	'b110101110:
	begin
		residulas_t4 = Multiplier_Float; //Gx*(-sinTheta*Dx - cosTheta*Dy)
		a = residuals_aff_cosTheta;
		b = residuals_aff_Dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110101111;		
	end
	'b110101111:
	begin
		residulas_t1 = Multiplier_Float;
		a = residuals_aff_sinTheta;
		b = residuals_aff_Dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110110000;
	end
	'b110110000:
	begin
		residulas_t2 = Multiplier_Float;
		a = residulas_t1;
		b = residulas_t2;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110110001;
	end
	'b110110001:
	begin
		residulas_t3 = Subtractor_Float;
		a = residuals_aff_Gy;
		b = residulas_t3;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110110010;
	end
	'b110110010:
	begin
		residulas_t5 = Multiplier_Float; //Gy*(cosTheta*Dx - sinTheta*Dy)
		a = residulas_t4;
		b = residulas_t5;
		state = 7'b1000000; //Adder
		temp_state = 'b110110011;
	end
	'b110110011:
	begin
		residuals_aff_delTheta = Adder_Float; //delTheta = Gx*(-sinTheta*Dx - cosTheta*Dy) + Gy*(cosTheta*Dx - sinTheta*Dy);
		a = residuals_aff_Gx;
		b = residuals_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110110100;
	end
	'b110110100:
	begin
		a = Multiplier_Float;
		b = residuals_aff_cosTheta;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110110101;
	end
	'b110110101:
	begin
		residulas_t1 = Multiplier_Float; //Gx*dx*cosTheta
		a = residuals_aff_Gy;
		b = residuals_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110110110;	
	end
	'b110110110:
	begin
		a = Multiplier_Float;
		b = residuals_aff_sinTheta;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110110111;
	end
	'b110110111:
	begin
		residulas_t2 = Multiplier_Float; //Gy*dx*sinTheta
		a = residulas_t1;
		b = residulas_t2;
		state = 7'b1000000; //Adder
		temp_state = 'b110111000;
	end
	'b110111000:
	begin
		residuals_aff_delEx = Adder_Float; //delEx = Gx*dx*cosTheta + Gy*dx*sinTheta;
		a = residuals_aff_Gx;
		b = residuals_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110111001;
	end
	'b110111001:
	begin
		a = Multiplier_Float;
		b = residuals_aff_sinTheta;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110111010;
	end
	'b110111010:
	begin
		residulas_t1 = Multiplier_Float; //Gx*dy*sinTheta 
		a = residuals_aff_Gy;
		b = residuals_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110111011;
	end
	'b110111011:
	begin
		a = Multiplier_Float;
		b = residuals_aff_cosTheta;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110111100;
	end
	'b110111100:
	begin
		residulas_t2 = Multiplier_Float; //Gy*dy*cosTheta
		a = residulas_t2;
		b = residulas_t1;
		state = 7'b1000101; //Subtractor
		temp_state = 'b110111101;
	end
	'b110111101:
	begin
		residuals_aff_delEy = Subtractor_Float; //delEy = -Gx*dy*sinTheta + Gy*dy*cosTheta;
		a = residuals_aff_cosTheta;
		b = residuals_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110111110;
	end
	'b110111110:
	begin
		residulas_t1 = Multiplier_Float; //cosTheta*dy 
		a = residuals_aff_sinTheta;
		b = residuals_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b110111111;
	end
	'b110111111:
	begin
		b = Multiplier_Float; //sinTheta*dx
		a = residulas_t1;		
		state = 7'b1000101; //Subtractor
		temp_state = 'b111000000;
	end
	'b111000000:
	begin
		a = Subtractor_Float; //(cosTheta*dy - sinTheta*dx)
		b = residuals_aff_Gx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b111000001;
	end
	'b111000001:
	begin
		residulas_t2 = Multiplier_Float; //Gx*(cosTheta*dy - sinTheta*dx)
		a = residuals_aff_sinTheta;
		b = residuals_aff_dy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b1110000100;
	end
	'b1110000100:
	begin
		residulas_t3 = Multiplier_Float;
		a = residuals_aff_cosTheta;
		b = residuals_aff_dx;
		state = 7'b1000110; //Multiplier
		temp_state = 'b1110000101;
	end
	'b1110000101:
	begin
		a = Multiplier_Float;
		b = residulas_t3;
		state = 7'b1000000; //Adder
		temp_state = 'b1110000110;
	end
	'b1110000110:
	begin
		a = Adder_Float; //(sinTheta*dy + cosTheta*dx)
		b = residuals_aff_Gy;
		state = 7'b1000110; //Multiplier
		temp_state = 'b1110000111;
	end
	'b1110000111:
	begin
		residulas_t4 = Multiplier_Float; //Gy*(sinTheta*dy + cosTheta*dx)
		a = residulas_t2;
		b = residulas_t4;
		state = 7'b1000000; //Adder
		temp_state = 'b1110001000;
	end
	'b1110001000:
	begin
		residuals_aff_delGxy = Adder_Float; //delGxy = Gx*(cosTheta*dy - sinTheta*dx) + Gy*(sinTheta*dy + cosTheta*dx);
		state = 'b1110001001;
	end
	'b1110001001:
	begin
		residuals_aff[31:0] = residuals_aff_Gx; //SUBSET_DISPLACEMENT_X_FS[gid] = residuals_aff_Gx;
		residuals_aff[63:32] = residuals_aff_Gy; //SUBSET_DISPLACEMENT_Y_FS[gid] = residuals_aff_Gy;
		residuals_aff[95:64] = residuals_aff_delTheta; //ROTATION_Z_FS[gid] = residuals_aff_delTheta;
		residuals_aff[127:96] = residuals_aff_delEx; //NORMAL_STRETCH_XX_FS[gid] = residuals_aff_delEx;
		residuals_aff[159:128] = residuals_aff_delEy; //NORMAL_STRETCH_YY_FS[gid] = residuals_aff_delEy;
		residuals_aff[191:160] = residuals_aff_delGxy; //SHEAR_STRETCH_XY_FS[gid] = residuals_aff_delGxy;
		state = residuals_aff_temp_state;
	end
	'b1110001010: //insert_motion_aff
	begin
		parameters_[0] = imaff_u; //SUBSET_DISPLACEMENT_X_FS[gid] = imaff_u;
		parameters_[1] = imaff_v; //SUBSET_DISPLACEMENT_Y_FS[gid] = imaff_v;
		parameters_[2] = imaff_theta; //ROTATION_Z_FS[gid] = imaff_theta;
		state = insert_motion_aff_temp_state;
	end
	'b1110001011: //test_for_convergence_aff
	begin
		a = parameters_[0];
		b = def_old[0];
		state = 7'b1000101; //Subtractor
		temp_state = 'b1110001100;
	end
	'b1110001100:
	begin
		if(less_than(fast_solver_tolerance, Subtractor_Float))
		begin
			converged = 1'b0;
			state = 'b1110001111;
		end
		else
		begin
			a = parameters_[1];
			b = def_old[1];
			state = 7'b1000101; //Subtractor
			temp_state = 'b1110001101;
		end
	end
	'b1110001101:
	begin
		if(less_than(fast_solver_tolerance, Subtractor_Float))
		begin
			converged = 1'b0;
			state = 'b1110001111;
		end
		else
		begin
			a = parameters_[2];
			b = def_old[2];
			state = 7'b1000101; //Subtractor
			temp_state = 'b1110001110;
		end
	end
	'b1110001110:
	begin
		if(less_than(fast_solver_tolerance, Subtractor_Float))
		begin
			converged = 1'b0;
			state = 'b1110001111;
		end
		else
		begin
			state = 'b1110001111;
		end
	end
	'b1110001111:
	begin
		state = test_for_convergence_aff_temp_state;
	end
    'b1110100000:   //save_fields
    begin
        SUBSET_DISPLACEMENT_X_FS[gid] = parameters_[0];
        SUBSET_DISPLACEMENT_Y_FS[gid] = parameters_[1];
        ROTATION_Z_FS[gid] = parameters_[2];
        NORMAL_STRETCH_XX_FS[gid] = parameters_[3];
        NORMAL_STRETCH_YY_FS[gid] = parameters_[4];
        SHEAR_STRETCH_XY_FS[gid] = parameters_[5];
        state = save_fields_temp_state;
    end
    //last statet b1110100101
    endcase
    
end //always
 
/////////////////////////////////////////////////// FUNCTIONS ///////////////////////////////////////////////////
function [31:0] subset_global_id_function;
    input [31:0] ind;
    begin
        subset_global_id_function = subset_global_id[ind];
    end
endfunction

///////////////////////////// Start of Closet_Triad() /////////////////////////////
function [31:0] closest_triad;
    input [31:0] u, v, t;
    input [31:0] id, distance_sqr;
    reg [31:0] query_pt [0:2];
    begin
        query_pt[0] = u;
        query_pt[1] = v;
        query_pt[2] = t;
        closest_triad = query_pt[0];
        //knsearch ???
    end
endfunction
///////////////////////////// End of Closet_Triad() /////////////////////////////

///////////////////////////// Start of Neighbor() /////////////////////////////
function [31:0] neighbor;
    input [31:0] triad_id;
    input [31:0] neighbor_index;
    reg [31:0] index;
    begin
        index = triad_id * num_neighbors_;
        index = index + neighbor_index;
        neighbor = neighbors_[index]; //**** define neighbors array
    end
endfunction
///////////////////////////// End of Neighbor() /////////////////////////////

///////////////////////////// Start of Less_than() /////////////////////////////
function less_than;
    input [31:0] a;
    input [31:0] b;
    begin
        if(a[31] == 1'b1 && b[31] == 1'b0)
        begin
            less_than = 1'b1;
        end
        else if(a[31] == 1'b0 && b[31] == 1'b1)
        begin
            less_than = 1'b0;
        end
        else
        begin
            if(a[30:0] < b[30:0])
            begin
                less_than = 1'b1;
            end
            else
            begin
                less_than = 1'b0;
            end
        end
    end
endfunction 
///////////////////////////// End of Less_than() /////////////////////////////

////////////////////////////// bit2bit_sq //////////////////////////////////////
function [51:0] bit2bit_sq;
  input [25:0] d;
  integer j;
  begin
       for (j = 25; j >= 0; j = j -1)
       begin
            // x'right must be zero
            bit2bit_sq[2*j]= d[j];
            bit2bit_sq[2*j+1] = 1'b0;
        end
  end
endfunction
////////////////////////////// End bit2bit_sq //////////////////////////////////////

endmodule
