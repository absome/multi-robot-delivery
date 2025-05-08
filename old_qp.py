    def compute_qp(self):
        u_gtg = self.compute_ugtg(caster=True)
        Q_mat = 2*cvxopt.matrix(np.eye(2), tc='d')
        c_mat = -2*cvxopt.matrix(u_gtg[:2], tc='d')
        
        H = []
        b = []
        # Test for sample obstacle in (0,0)
        caster_pos = self.caster_xy()
        obs_xy = np.array([0.,0.])
        # h_avo = np.linalg.norm(caster_pos - obs_xy)**2 - 1**2
        h_avo = -2*(caster_pos - obs_xy).T
        H.append(h_avo)
        g_avo = 2
        b.append(g_avo*h_avo)
        
        # Get h(s) values for each robot and obstacle
        for r in self.other_robots:
            error = self.state[:2] - r.state[:2] # this robot distance to r robot
            dij = 1
            epsilon = 0.1
            g_form = 2
            h1 = -np.linalg.norm(error)**2 + (dij + epsilon)**2
            b.append(g_form*h1)
            h2 = np.linalg.norm(error)**2 - (dij - epsilon)**2
            b.append(g_form*h2)

            
        H_mat = matrix(np.vstack(H), tc='d')
        b_mat = matrix(np.array(b).reshape(-1, 1), tc='d')
        
        # Solve the optimization problem
        solvers.options['show_progress'] = False
        sol = solvers.qp(Q_mat, c_mat, H_mat, b_mat, verbose=False)
        
        caster_input = np.array([sol['x'][0], sol['x'][1], 0])
        
        # convert from caster to robot input
        ux, uy = caster_input[:]
        theta = self.state[2]
        l = self.caster_distance
        v = np.cos(theta)*ux + np.sin(theta)*uy
        w = (-np.sin(theta)/l)*ux + (np.cos(theta)/l)*uy
        current_input = np.array([v, w])
        
        return current_input
    
    
    
    
        def compute_qp(self):
        u_gtg = self.compute_ugtg(caster=True)
        Q_mat = 2*cvxopt.matrix(np.eye(2), tc='d')
        c_mat = -2*cvxopt.matrix(u_gtg[:2], tc='d')
        
        H = np.zeros([1, 2])
        b = np.zeros([1, 1])
        # Test for sample obstacle in (0,0)
        caster_pos = self.caster_xy()
        obs_xy = np.array([0.,0.])
        # h_avo = np.linalg.norm(caster_pos - obs_xy)**2 - 1**2
        h1 = -2*(caster_pos - obs_xy).T
        H[0,:] = h1
        
        gamma = 0.2
        h = np.linalg.norm(self.state[:2] - obs_xy)**2 - (0.5)**2
        b[0,:] = gamma*h

            
        H_mat = cvxopt.matrix(H, tc='d')
        b_mat = cvxopt.matrix(b, tc='d')
        
        # Solve the optimization problem
        cvxopt.solvers.options['show_progress'] = False
        sol = cvxopt.solvers.qp(Q_mat, c_mat, H_mat, b_mat, verbose=False)        
        caster_input = np.array([sol['x'][0], sol['x'][1], 0])
        print(caster_input)
        # convert from caster to robot input
        ux, uy = caster_input[:2]
        theta = self.state[2]
        l = self.caster_distance
        cos_theta, sin_theta = np.cos(theta), np.sin(theta)
        v = np.cos(theta)*ux + np.sin(theta)*uy
        w = (-np.sin(theta)/l)*ux + (np.cos(theta)/l)*uy
        current_input = np.array([v, w])
        
        return current_input*5
    
    
        def compute_qp(self):
        neighbours = self.neighbouring_robots          # exclude self!
        obs_pos = np.array([0., 0.5])
        Rs = 1.0

        d_ref = 2.0
        eps   = 1.0          # << tighter tolerance
        g_form = 0.5
        g_avo  = 0.2

        # 1. cost -------------------------------------------------------------
        u_gtg  = self.compute_ugtg(caster=True)[:2]
        Q_mat  = cvxopt.matrix(2*np.eye(2), tc='d')
        c_mat  = cvxopt.matrix(-2*u_gtg.reshape(2,1), tc='d')

        # 2. barriers ---------------------------------------------------------
        n_rows = len(neighbours)*2 + 1
        H = np.zeros((n_rows, 2))
        b = np.zeros((n_rows, 1))
        row = 0

        s_i = self.caster_xy()          # (2,)

        # -- formation rows
        for r in neighbours:
            s_j = r.caster_xy()
            diff = s_i - s_j
            dij2 = diff @ diff

            # upper distance
            h1 = -(dij2) + (d_ref+eps)**2
            H[row,:] = -2*diff
            b[row,0] = (g_form/2)*h1
            row += 1

            # lower distance
            h2 =  dij2  - (d_ref-eps)**2
            H[row,:] =  2*diff
            b[row,0] = (g_form/2)*h2
            row += 1

        # -- obstacle row
        diff_o = s_i - obs_pos
        h_obs  = diff_o @ diff_o - Rs**2
        H[row,:] = -2*diff_o
        b[row,0] = g_avo * h_obs

        assert row == n_rows-1, "row counter mismatch!"

        # 3. solve ------------------------------------------------------------
        sol = cvxopt.solvers.qp(Q_mat, c_mat,
                                cvxopt.matrix(H, tc='d'),
                                cvxopt.matrix(b, tc='d'))
        ux, uy = np.array(sol['x']).flatten()

        # 4. map to (v, ω) ----------------------------------------------------
        theta = self.state[2]
        L     = self.caster_distance
        v     =  np.cos(theta)*ux + np.sin(theta)*uy
        w     = (-np.sin(theta)/L)*ux + (np.cos(theta)/L)*uy

        return np.array([v, w])