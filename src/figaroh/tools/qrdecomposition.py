"""QR decomposition utilities for robot parameter identification."""

from typing import Dict, List, Optional, Tuple, Union
import numpy as np
from scipy import linalg

# Constants
TOL_QR = 1e-8
TOL_BETA = 1e-6


class QRDecomposer:
    """Enhanced QR decomposition handler for robot parameter identification."""
    
    def __init__(self, tolerance: float = TOL_QR, beta_tolerance: float = TOL_BETA):
        self.tolerance = tolerance
        self.beta_tolerance = beta_tolerance
    
    def decompose_with_pivoting(
        self, 
        tau: np.ndarray, 
        W_e: np.ndarray, 
        params_r: List[str]
    ) -> Tuple[np.ndarray, Dict[str, float]]:
        """QR decomposition with column pivoting."""
        Q, R, P = linalg.qr(W_e, pivoting=True)
        
        # Reorder parameters according to pivoting
        params_sorted = [params_r[P[i]] for i in range(P.shape[0])]
        
        # Find effective rank
        rank = self._find_rank(R)
        
        # Extract base components
        R1, Q1, R2 = self._extract_base_components(R, Q, rank)
        
        # Compute base parameters
        beta = np.around(np.linalg.solve(R1, R2), 6)
        phi_b = np.round(np.linalg.solve(R1, Q1.T @ tau), 6)
        W_b = Q1 @ R1
        
        # Build parameter expressions
        base_params = self._build_parameter_expressions(
            params_sorted[:rank], params_sorted[rank:], beta
        )
        
        return W_b, dict(zip(base_params, phi_b))
    
    def double_decomposition(
        self,
        tau: np.ndarray,
        W_e: np.ndarray, 
        params_r: List[str],
        params_std: Optional[Dict[str, float]] = None
    ) -> Union[Tuple[np.ndarray, Dict, List, np.ndarray], 
               Tuple[np.ndarray, Dict, List, np.ndarray, np.ndarray]]:
        """Double QR decomposition for base parameter identification."""
        
        # First QR to identify base parameters
        base_indices, regroup_indices = self._identify_base_parameters(W_e, params_r)
        
        # Regroup and second QR
        W_base, W_regroup, params_base, params_regroup = self._regroup_parameters(
            W_e, params_r, base_indices, regroup_indices
        )
        
        # Second QR decomposition
        W_regrouped = np.c_[W_base, W_regroup]
        Q_r, R_r = np.linalg.qr(W_regrouped)
        
        rank = len(base_indices)
        R1, Q1, R2 = self._extract_base_components(R_r, Q_r, rank)
        
        # Compute parameters
        beta = np.around(np.linalg.solve(R1, R2), 6)
        phi_b = np.round(np.linalg.solve(R1, Q1.T @ tau), 6)
        W_b = Q1 @ R1
        
        # Verify consistency
        assert np.allclose(W_base, W_b), "Base regressor calculation error"
        
        # Build expressions and compute standard parameters if provided
        params_base_expr = self._build_parameter_expressions(
            params_base, params_regroup, beta
        )
        
        base_parameters = dict(zip(params_base_expr, phi_b))
        
        if params_std is not None:
            phi_std = self._compute_standard_parameters(
                params_base, params_regroup, beta, params_std
            )
            return W_b, base_parameters, params_base_expr, phi_b, phi_std
        
        return W_b, base_parameters, params_base_expr, phi_b
    
    def _find_rank(self, R: np.ndarray) -> int:
        """Find effective rank of R matrix."""
        diag_R = np.abs(np.diag(R))
        rank_indices = np.where(diag_R > self.tolerance)[0]
        return len(rank_indices) if len(rank_indices) > 0 else R.shape[0]
    
    def _extract_base_components(
        self, R: np.ndarray, Q: np.ndarray, rank: int
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Extract base components from QR decomposition."""
        R1 = R[:rank, :rank]
        Q1 = Q[:, :rank]
        R2 = R[:rank, rank:] if rank < R.shape[1] else np.array([]).reshape(rank, 0)
        return R1, Q1, R2
    
    def _identify_base_parameters(
        self, W_e: np.ndarray, params_r: List[str]
    ) -> Tuple[List[int], List[int]]:
        """Identify base and regrouped parameter indices."""
        Q, R = np.linalg.qr(W_e)
        diag_R = np.abs(np.diag(R))
        
        base_indices = [i for i, val in enumerate(diag_R) if val > self.tolerance]
        regroup_indices = [i for i, val in enumerate(diag_R) if val <= self.tolerance]
        
        return base_indices, regroup_indices
    
    def _regroup_parameters(
        self, W_e: np.ndarray, params_r: List[str], 
        base_indices: List[int], regroup_indices: List[int]
    ) -> Tuple[np.ndarray, np.ndarray, List[str], List[str]]:
        """Regroup parameters into base and dependent sets."""
        W_base = W_e[:, base_indices]
        W_regroup = W_e[:, regroup_indices] if regroup_indices else np.array([]).reshape(W_e.shape[0], 0)
        
        params_base = [params_r[i] for i in base_indices]
        params_regroup = [params_r[i] for i in regroup_indices]
        
        return W_base, W_regroup, params_base, params_regroup
    
    def _build_parameter_expressions(
        self, base_params: List[str], regroup_params: List[str], beta: np.ndarray
    ) -> List[str]:
        """Build parameter expressions including dependencies."""
        expressions = base_params.copy()
        
        for i, base_param in enumerate(expressions):
            for j, regroup_param in enumerate(regroup_params):
                if j < beta.shape[1] and abs(beta[i, j]) > self.beta_tolerance:
                    sign = " - " if beta[i, j] < 0 else " + "
                    coefficient = abs(beta[i, j])
                    expressions[i] += f"{sign}{coefficient}*{regroup_param}"
        
        return expressions
    
    def _compute_standard_parameters(
        self, base_params: List[str], regroup_params: List[str], 
        beta: np.ndarray, params_std: Dict[str, float]
    ) -> np.ndarray:
        """Compute standard parameter values."""
        phi_std = [params_std[param] for param in base_params]
        
        for i in range(len(phi_std)):
            for j, regroup_param in enumerate(regroup_params):
                if j < beta.shape[1]:
                    phi_std[i] += beta[i, j] * params_std[regroup_param]
        
        return np.around(phi_std, 5)


# Backward compatibility functions
def QR_pivoting(tau: np.ndarray, W_e: np.ndarray, params_r: list, tol_qr: float = TOL_QR) -> tuple:
    """Legacy QR pivoting function for backward compatibility."""
    decomposer = QRDecomposer(tolerance=tol_qr)
    return decomposer.decompose_with_pivoting(tau, W_e, params_r)


def double_QR(tau: np.ndarray, W_e: np.ndarray, params_r: list, 
              params_std: dict = None, tol_qr: float = TOL_QR) -> tuple:
    """Legacy double QR function for backward compatibility."""
    decomposer = QRDecomposer(tolerance=tol_qr)
    return decomposer.double_decomposition(tau, W_e, params_r, params_std)


def get_baseParams(W_e: np.ndarray, params_r: list, params_std: dict = None, 
                  tol_qr: float = TOL_QR) -> tuple:
    """Legacy function for getting base parameters."""
    decomposer = QRDecomposer(tolerance=tol_qr)
    dummy_tau = np.zeros(W_e.shape[0])  # Not used in this function
    result = decomposer.double_decomposition(dummy_tau, W_e, params_r, params_std)
    return result[0], result[2], decomposer._identify_base_parameters(W_e, params_r)[0]


def get_baseIndex(W_e: np.ndarray, params_r: list, tol_qr: float = TOL_QR) -> tuple:
    """Legacy function for getting base indices."""
    decomposer = QRDecomposer(tolerance=tol_qr)
    base_indices, _ = decomposer._identify_base_parameters(W_e, params_r)
    return tuple(base_indices)


def build_baseRegressor(W_e: np.ndarray, idx_base: tuple) -> np.ndarray:
    """Legacy function for building base regressor."""
    return W_e[:, list(idx_base)]


def cond_num(W_b: np.ndarray, norm_type: str = None) -> float:
    """Calculate condition number with various norms."""
    if norm_type == "fro":
        return np.linalg.cond(W_b, "fro")
    elif norm_type == "max_over_min_sigma":
        return np.linalg.cond(W_b, 2) / np.linalg.cond(W_b, -2)
    else:
        return np.linalg.cond(W_b)
